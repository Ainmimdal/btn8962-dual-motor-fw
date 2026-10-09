"""CAN coexistence test against the simulated board with a fake ODrive on the bus.

    sh run.sh --odrive &          # board at nodes 40/41, fake ODrive 0.5.6 at node 0
    python3 test_can_odrive.py    # needs python-can (and pyserial for slcan)

Drives the board with plain ODrive CAN Simple frames, the way ODrive host code
would, and checks that frames for the ODrive leave the board alone.
"""
import struct
import sys
import time

import can

N1, N2, OD = 40, 41, 0
bus = can.Bus(interface="slcan", channel=sys.argv[1] if len(sys.argv) > 1 else "/tmp/mdsim-can",
              bitrate=125000)
fails = 0


def check(cond, msg):
    global fails
    print(("ok:   " if cond else "FAIL: ") + msg)
    fails += 0 if cond else 1


def send(node, cmd, data=b"", rtr=False):
    bus.send(can.Message(arbitration_id=node << 5 | cmd, data=bytes(data), is_extended_id=False,
                         is_remote_frame=rtr, dlc=8 if rtr else len(data)))


def collect(secs, keepalive=True):
    out, end, nxt = [], time.time() + secs, 0
    while time.time() < end:
        if keepalive and time.time() > nxt:   # any frame to the node feeds the watchdog
            send(N1, 0x01, rtr=True)
            nxt = time.time() + 0.1
        m = bus.recv(0.02)
        if m is not None and not m.is_remote_frame:
            out.append((m.arbitration_id >> 5, m.arbitration_id & 0x1F, bytes(m.data)))
    return out


def last(frames, node, cmd):
    hits = [d for n, c, d in frames if n == node and c == cmd]
    return hits[-1] if hits else None


def heartbeat(frames, node):
    d = last(frames, node, 0x01)
    return struct.unpack("<IB", d[:5]) if d else (None, None)


f32 = lambda *v: struct.pack("<" + "f" * len(v), *v)
i32 = lambda *v: struct.pack("<" + "i" * len(v), *v)

# 1. who is on the bus
send(N1, 0x1E, bytes([0x05]))       # vendor INFO
fr = collect(1.0)
info = next((d for n, c, d in fr if n == N1 and c == 0x1F and d[0] == 5), None)
check(info is not None, f"vendor INFO: fw {info[1]}.{info[2]}.{info[3]}, current sense {info[5]}" if info else "vendor INFO")
has_cs = bool(info and info[5])
nodes = {n for n, _, _ in fr}
check(nodes == {OD, N1, N2}, f"nodes seen on the bus: {sorted(nodes)}")
check({c for n, c, _ in fr if n in (N1, N2)} <= {0x01, 0x09, 0x1F}, "board only sends heartbeat, estimates, vendor status")
check(heartbeat(fr, N1) == (0, 1) and heartbeat(fr, OD)[1] == 1, "board and ODrive heartbeats both IDLE")

# 2. commands for the ODrive don't touch the board
send(OD, 0x0B, i32(2, 1))
send(OD, 0x07, i32(8))
send(OD, 0x0D, f32(2.0, 0.0))
fr = collect(0.6)
check(heartbeat(fr, OD)[1] == 8 and heartbeat(fr, N1)[1] == 1 and heartbeat(fr, N2)[1] == 1,
      "ODrive went closed-loop, board motors stayed IDLE")
ov = struct.unpack("<ff", last(fr, OD, 0x09))[1]
check(abs(ov - 2.0) < 1e-6, f"ODrive runs at its own velocity input ({ov})")

# 3. velocity control of motor 1, ODrive style
send(N1, 0x0B, i32(2, 1))           # VELOCITY_CONTROL, PASSTHROUGH
send(N1, 0x07, i32(8))              # CLOSED_LOOP_CONTROL
send(N1, 0x0D, f32(3000.0, 0.0))    # turns/s (ENC1_CPR = 1, so counts/s)
fr = collect(1.5)
pos, vel = struct.unpack("<ff", last(fr, N1, 0x09))
check(heartbeat(fr, N1) == (0, 8), "motor 1 heartbeat: CLOSED_LOOP, no error")
check(abs(vel - 3000) < 150, f"motor 1 velocity via 0x0D: {vel:.0f}")
check(heartbeat(fr, N2)[1] == 1, "motor 2 untouched")

# 4. gets by RTR and by DLC 0, replies on the same ID
send(N2, 0x09, rtr=True)
send(N1, 0x0A)
send(N1, 0x14, rtr=True)
fr = collect(0.2)
check(last(fr, N2, 0x09) is not None, "RTR get of encoder estimates on motor 2")
cnt = last(fr, N1, 0x0A)
check(cnt is not None and abs(struct.unpack("<i", cnt[:4])[0] - pos) < 600, "DLC-0 get of encoder count")
iq = last(fr, N1, 0x14)
iq_meas = struct.unpack("<ff", iq)[1] if iq else None
check(iq is not None and (iq_meas == iq_meas) == has_cs,
      "Iq measured is " + ("a number" if has_cs else "NaN (no current sense), never 0"))

# 5. position control of motor 2
send(N2, 0x0B, i32(3, 1))
send(N2, 0x07, i32(8))
send(N2, 0x0C, f32(5000.0) + struct.pack("<hh", 0, 0))
fr = collect(3.0)
p2 = struct.unpack("<ff", last(fr, N2, 0x09))[0]
check(abs(p2 - 5000) < 20, f"motor 2 position via 0x0C: {p2:.0f}")
hb = last(fr, N2, 0x01)
check(hb[7] & 0x80, "traj_done set once in position")

# 6. estop latches, clear errors releases
send(N1, 0x02)
fr = collect(0.3)
check(heartbeat(fr, N1) == (0x4000, 1), f"estop -> ESTOP_REQUESTED, IDLE {heartbeat(fr, N1)}")
send(N1, 0x07, i32(8))
fr = collect(0.3)
check(heartbeat(fr, N1)[1] == 1, "closed loop refused while estopped")
send(N1, 0x18)
send(N1, 0x07, i32(3))              # FULL_CALIBRATION_SEQUENCE: not on a DC drive
fr = collect(0.3)
check(heartbeat(fr, N1) == (0x1, 1), f"cleared, then calibration -> INVALID_STATE {heartbeat(fr, N1)}")
send(N1, 0x18)
send(N2, 0x07, i32(1))

# 7. watchdog: go quiet with a motor on
send(N1, 0x0B, i32(2, 1))
send(N1, 0x07, i32(8))
time.sleep(0.05)
fr = collect(1.0, keepalive=False)
check(heartbeat(fr, N1) == (0x800, 1), f"no traffic -> WATCHDOG_TIMER_EXPIRED, IDLE {heartbeat(fr, N1)}")
send(N1, 0x18)

# 8. vendor params and node ID change
send(N1, 0x1E, bytes([0x02, 29]))
fr = collect(0.2)
r = next((d for n, c, d in fr if n == N1 and c == 0x1F and d[0] == 2), None)
check(r and r[0] == 2 and r[1] == 29 and struct.unpack("<f", r[4:])[0] == 40.0, "vendor GET_PARAM M1_NODE_ID = 40")
send(N1, 0x1E, bytes([0x01, 30]) + f32(40.0))
fr = collect(0.2)
r = [d for n, c, d in fr if c == 0x1F and d[0] == 1]
check(r and r[-1][2] == 2, "M2_NODE_ID = M1_NODE_ID refused")
send(N1, 0x1E, bytes([0x01, 29]) + f32(20.0))
collect(0.2, keepalive=False)       # let frames already in flight drain
fr = collect(0.5, keepalive=False)
check(20 in {n for n, _, _ in fr} and N1 not in {n for n, _, _ in fr}, "motor 1 moved to node 20")
send(20, 0x06, i32(N1))             # ODrive "set node id" back to 40
collect(0.2)
fr = collect(0.5)
check(N1 in {n for n, _, _ in fr} and 20 not in {n for n, _, _ in fr}, "0x06 set node id back to 40")

# 9. 0.5.1 heartbeat layout
send(N1, 0x1E, bytes([0x01, 34]) + f32(51.0))
fr = collect(0.3)
hb = last(fr, N1, 0x01)
check(struct.unpack("<II", hb) == (0, 1), "CAN_ODRV_VER 51: u32 error, u32 state")
send(N1, 0x1E, bytes([0x01, 34]) + f32(56.0))
send(N1, 0x1E, bytes([0x09]))

bus.shutdown()
print("CAN ODRIVE TEST " + ("FAILED" if fails else "PASSED"))
sys.exit(1 if fails else 0)
