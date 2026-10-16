"""Board links: ASCII protocol over a serial port, or the ODrive-style
CAN Simple protocol (plus vendor commands 0x1E/0x1F) over any python-can
interface. The CAN link only talks to the board's two node IDs, so ODrives
on the same bus are left alone. Both run a reader thread and push events onto
a queue that the GUI drains:

  ("telem", motor_index, dict)   pos, vel, cur (None = unavailable), duty, mode, faults, limit
  ("param", id, value, status)   status "" on success, else an error word
  ("paramdef", id, name, value, min, max, default)   UART only, from "CFG"
  ("info", dict)                 fw, enc1, cs, params
  ("event", text)                faults, watchdog, errors from the board
  ("log", text)                  raw traffic for the console
  ("closed", reason)
"""
import queue
import struct
import threading
import time

from params import MODES, BY_ID, UNIT_PARAMS, counts_per_turn

PARAM_STATUS = {0: "", 1: "BAD_ID", 2: "RANGE", 3: "BUSY", 4: "FLASH"}


class Link:
    def __init__(self, q: queue.Queue):
        self.q = q
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()

    def emit(self, *ev):
        self.q.put(ev)


# --------------------------------------------------------------------- UART
class UartLink(Link):
    kind = "UART"

    def __init__(self, q, port, baud=115200):
        import serial
        super().__init__(q)
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._wlock = threading.Lock()
        self.start()

    def close(self):
        super().close()
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, line, log=True):
        with self._wlock:
            self.ser.write((line + "\n").encode())
        if log:
            self.emit("log", "> " + line)

    # commands
    def set_mode(self, m, mode): self.send(f"MODE {m + 1} {MODES[mode]}")
    def set_target(self, m, v): self.send(f"SET {m + 1} {v:.6g}")
    def clear(self, m=None): self.send("CLR ALL" if m is None else f"CLR {m + 1}")
    def zero(self, m): self.send(f"ZERO {m + 1}")
    def home(self, m): self.send(f"HOME {m + 1}")
    def stop(self): self.send("STOP")
    def ping(self): self.send("PING", log=False)
    def info(self): self.send("VER")
    def read_param(self, pid): self.send(f"CFG {pid}")
    def read_all(self): self.send("CFG")
    def write_param(self, pid, v): self.send(f"CFG {pid} {v:.9g}")
    def save(self): self.send("SAVE")
    def defaults(self):
        self.send("DEFAULTS")
        self.send("CFG")
    def raw(self, text): self.send(text)

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # unplugged
                self.emit("closed", str(e))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text:
                    self._line(text)

    def _line(self, t):
        try:
            if t.startswith("T,"):
                self._telem(t)
                return
            if t == "OK PONG":
                return
            if t.startswith("C ") or t.startswith("OK C "):
                f = t.split()[-7:]   # id name value min max default flags
                pid, name = int(f[0]), f[1]
                val, lo, hi, de = (float(x) for x in f[2:6])
                self.emit("paramdef", pid, name, val, lo, hi, de)
                self.emit("param", pid, val, "")
                if t.startswith("OK"):
                    self.emit("log", "< " + t)
                return
            if t.startswith("OK FW="):
                kv = dict(x.split("=", 1) for x in t.split()[1:])
                self.emit("info", {"fw": kv.get("FW"), "enc1": kv.get("ENC1"),
                                   "cs": kv.get("CS") == "1", "params": int(kv.get("PARAMS", 0))})
            elif t.startswith("EVT") or t.startswith("ERR"):
                self.emit("event", t)
            elif t.startswith("OK SAVED") or t.startswith("OK DEFAULTS") or t.startswith("OK LIMIT"):
                self.emit("event", t)
            self.emit("log", "< " + t)
        except (ValueError, IndexError):
            self.emit("log", "< " + t + "   (unparsed)")

    def _telem(self, t):
        f = t.split(",")
        # T,ms, then 6 fields per motor, then ppm1, ppm2[, limit bits[, home bits]]
        lim = int(f[16]) if len(f) > 16 else 0
        home = int(f[17]) if len(f) > 17 else 0
        for m in range(2):
            pos, vel, cur, duty, mode, faults = f[2 + 6 * m: 8 + 6 * m]
            self.emit("telem", m, {
                "pos": int(pos), "vel": float(vel),
                "cur": None if cur == "NA" else int(cur) / 1000.0,
                "duty": int(duty) / 1000.0,
                "mode": "ODVP".index(mode), "faults": int(faults, 16),
                "limit": bool(lim >> m & 1),
                "homing": bool(home >> m & 1), "homed": bool(home >> (2 + m) & 1),
            })
        self.emit("ppm", int(f[14]), int(f[15]))


# ---------------------------------------------------------------------- CAN
# ODrive CAN Simple command IDs used here (ID = node << 5 | cmd)
OD_HEARTBEAT, OD_ESTIMATES, OD_CLEAR, VENDOR_REQ, VENDOR_TX = 0x01, 0x09, 0x18, 0x1E, 0x1F
V_SET, V_GET, V_SAVE, V_DEFAULTS, V_INFO, V_MODE, V_TARGET, V_ZERO, V_STOP, V_HOME = range(1, 11)
V_STATUS = 0x80
NODE_PARAMS = {29: 0, 30: 1}      # M1_NODE_ID, M2_NODE_ID


class CanLink(Link):
    kind = "CAN"

    def __init__(self, q, interface, channel, bitrate=125000, nodes=(40, 41)):
        import can
        super().__init__(q)
        self.nodes = list(nodes)
        self.units = {}               # encoder/gear params, for ODrive turns -> counts
        kwargs = {"interface": interface, "bitrate": bitrate}
        if channel:
            kwargs["channel"] = channel
        self.bus = can.Bus(**kwargs)
        self._can = can
        self._kin = [None, None]
        self._others = set()
        self.start()

    def close(self):
        super().close()
        time.sleep(0.15)
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _send(self, m, cmd, data=b"", rtr=False, log=True):
        ident = self.nodes[m] << 5 | cmd
        msg = self._can.Message(arbitration_id=ident, data=bytes(data), is_extended_id=False,
                                is_remote_frame=rtr, dlc=8 if rtr else len(data))
        try:
            self.bus.send(msg, timeout=0.2)
        except self._can.CanError as e:
            self.emit("event", f"CAN send failed: {e}")
            return
        if log:
            self.emit("log", f"> {ident:03X}#{'R' if rtr else bytes(data).hex().upper()}")

    def _vendor(self, m, sub, payload=b"", log=True):
        self._send(m, VENDOR_REQ, bytes([sub]) + bytes(payload), log=log)

    def set_mode(self, m, mode): self._vendor(m, V_MODE, [mode])
    def set_target(self, m, v): self._vendor(m, V_TARGET, struct.pack("<f", v))
    def zero(self, m): self._vendor(m, V_ZERO)
    def home(self, m): self._vendor(m, V_HOME)
    def stop(self): self._vendor(0, V_STOP)
    def ping(self): self._send(0, OD_HEARTBEAT, rtr=True, log=False)
    def info(self): self._vendor(0, V_INFO)
    def read_param(self, pid): self._vendor(0, V_GET, [pid])
    def write_param(self, pid, v): self._vendor(0, V_SET, bytes([pid]) + struct.pack("<f", v))
    def save(self): self._vendor(0, V_SAVE)

    def clear(self, m=None):
        for i in ([0, 1] if m is None else [m]):
            self._send(i, OD_CLEAR)

    def read_all(self):
        def worker():
            for pid in sorted(BY_ID):
                self._vendor(0, V_GET, [pid], log=False)
                time.sleep(0.01)
            self.emit("log", f"> requested {len(BY_ID)} parameters")
        threading.Thread(target=worker, daemon=True).start()

    def defaults(self):
        self._vendor(0, V_DEFAULTS)
        time.sleep(0.05)
        self.read_all()

    def raw(self, text):
        """'id#hexdata', or 'id#R' for a remote (request) frame, e.g. 201#R"""
        ident, _, data = text.partition("#")
        rtr = data.strip().upper() == "R"
        msg = self._can.Message(arbitration_id=int(ident, 16), is_extended_id=False,
                                is_remote_frame=rtr,
                                data=b"" if rtr else bytes.fromhex(data), dlc=8 if rtr else None)
        self.bus.send(msg, timeout=0.2)
        self.emit("log", "> " + text)

    def _reader(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(0.1)
            except Exception as e:
                if not self._stop.is_set():
                    self.emit("closed", str(e))
                return
            if msg is None or msg.is_extended_id or msg.is_remote_frame:
                continue
            node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
            if node in self.nodes:
                self._frame(self.nodes.index(node), cmd, bytes(msg.data))
            elif node not in self._others:
                self._others.add(node)
                self.emit("event", f"Other CAN node {node} on the bus (ID 0x{msg.arbitration_id:03X})")

    def _frame(self, m, cmd, d):
        if cmd == OD_ESTIMATES and len(d) >= 8:
            pos, vel = struct.unpack("<ff", d[:8])
            cpt = counts_per_turn(self.units, m)
            self._kin[m] = (round(pos * cpt), vel * cpt)
            return
        if cmd == OD_HEARTBEAT:
            return
        if cmd != VENDOR_TX or len(d) < 8:
            self.emit("log", f"< {self.nodes[m] << 5 | cmd:03X}#{d.hex().upper()}")
            return
        sub = d[0]
        if sub == V_STATUS:
            cur, duty = struct.unpack("<hh", d[4:8])
            pos, vel = self._kin[m] or (0, 0.0)
            self.emit("telem", m, {
                "pos": pos, "vel": float(vel),
                "cur": None if cur == -32768 else cur / 1000.0,
                "duty": duty / 1000.0, "mode": d[1], "faults": d[2], "limit": bool(d[3] & 1),
                "homing": bool(d[3] & 2), "homed": bool(d[3] & 4),
            })
            return
        if sub == V_INFO:
            enc = {1: "TIM1", 2: "TIM3_3V3"}.get(d[4], str(d[4]))
            self.emit("info", {"fw": f"{d[1]}.{d[2]}.{d[3]}", "enc1": enc,
                               "cs": bool(d[5]), "params": d[6]})
        else:
            status = PARAM_STATUS.get(d[2], f"ERR{d[2]}")
            val = struct.unpack("<f", d[4:8])[0]
            if sub in (V_SET, V_GET):
                pid = d[1]
                if not status:
                    if pid in UNIT_PARAMS:
                        self.units[pid] = val
                    if pid in NODE_PARAMS:
                        self.nodes[NODE_PARAMS[pid]] = int(val)
                self.emit("param", pid, val, status)
                if status:
                    name = BY_ID.get(pid, (pid, f"#{pid}"))[1]
                    self.emit("event", f"ERR {status} setting {name}")
            elif sub == V_SAVE:
                self.emit("event", "OK SAVED" if not status else f"ERR {status} on save")
            elif sub == V_DEFAULTS:
                self.emit("event", "OK DEFAULTS" if not status else f"ERR {status} on defaults")
        self.emit("log", f"< {self.nodes[m] << 5 | VENDOR_TX:03X}#{d.hex().upper()}")


def list_serial_ports():
    try:
        from serial.tools import list_ports
        # USB-serial adapters first; Black Magic Probe (VID 0x1D50) and non-USB
        # ports (Bluetooth, built-in) last, so the default pick is the adapter.
        ports = sorted(list_ports.comports(), key=lambda p: (p.vid is None or p.vid == 0x1D50))
        return [p.device for p in ports]
    except Exception:
        return []
