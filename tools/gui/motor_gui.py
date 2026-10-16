#!/usr/bin/env python3
"""Control and configuration GUI for the dual brushed motor driver.

Connects through a USB-UART adapter (ASCII protocol on USART3) or a USB-CAN
adapter (any python-can interface; 125 kbit/s). See README.md.

    pip install -r requirements.txt
    python motor_gui.py
"""
import collections
import json
import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from links import CanLink, UartLink, list_serial_ports
from params import BY_ID, BY_NAME, MODES, PARAMS, UNIT_PARAMS, counts_per_turn, fault_text

CAN_INTERFACES = ["slcan", "gs_usb", "pcan", "socketcan", "kvaser", "ixxat", "vector",
                  "canalystii", "seeedstudio", "usb2can", "serial", "virtual"]
# Position/velocity are in output degrees/RPM once ENCx_CPR (and the gear
# ratio) are set, raw counts while counts per output turn is still 1.
PLOT_SIGNALS = {"Velocity": "vel_u", "Position": "pos_u",
                "Current (A)": "cur", "Duty": "duty"}
PLOT_SECONDS = 10.0
KEEPALIVE_S = 0.2
MOTOR_COLORS = ["#2b6cb0", "#c05621"]


class MotorPanel(ttk.LabelFrame):
    def __init__(self, master, app, m):
        super().__init__(master, text=f"Motor {m + 1}", padding=8)
        self.app, self.m = app, m
        self.mode = tk.IntVar(value=0)
        self.vals = {k: tk.StringVar(value="-") for k in
                     ("mode", "pos", "vel", "cur", "duty", "faults", "limit", "home")}

        grid = ttk.Frame(self)
        grid.grid(row=0, column=0, columnspan=2, sticky="ew")
        labels = [("Mode", "mode"), ("Position", "pos"), ("Velocity", "vel"),
                  ("Current", "cur"), ("Duty", "duty"), ("Faults", "faults"), ("Limit", "limit"),
                  ("Home", "home")]
        for i, (text, key) in enumerate(labels):
            ttk.Label(grid, text=text + ":").grid(row=i, column=0, sticky="w")
            lbl = ttk.Label(grid, textvariable=self.vals[key], width=16, font=("TkFixedFont", 11))
            lbl.grid(row=i, column=1, sticky="w", padx=6)
            if key == "faults":
                self.fault_lbl = lbl

        modes = ttk.Frame(self)
        modes.grid(row=1, column=0, columnspan=2, pady=(8, 4), sticky="w")
        for i, name in enumerate(MODES):
            ttk.Radiobutton(modes, text=name, value=i, variable=self.mode,
                            command=self.on_mode).pack(side="left", padx=2)

        tgt = ttk.Frame(self)
        tgt.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(tgt, text="Target:").pack(side="left")
        self.target = tk.StringVar(value="0")
        e = ttk.Entry(tgt, textvariable=self.target, width=12)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda _e: self.on_send())
        ttk.Button(tgt, text="Send", command=self.on_send).pack(side="left")
        self.unit = ttk.Label(tgt, text="")
        self.unit.pack(side="left", padx=4)

        self.slider_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(self, from_=-1.0, to=1.0, variable=self.slider_var,
                                command=self.on_slider)
        self.slider.grid(row=3, column=0, columnspan=2, sticky="ew", pady=4)
        self._last_slider_send = 0.0

        btns = ttk.Frame(self)
        btns.grid(row=4, column=0, columnspan=2, sticky="w")
        ttk.Button(btns, text="Zero position", command=lambda: app.cmd("zero", m)).pack(side="left")
        ttk.Button(btns, text="Clear fault", command=lambda: app.cmd("clear", m)).pack(side="left", padx=4)
        ttk.Button(btns, text="Home", command=lambda: app.cmd("home", m)).pack(side="left")
        self.columnconfigure(0, weight=1)
        self._set_slider_range()

    def _set_slider_range(self):
        mode = self.mode.get()
        cpt, scaled = self.app.counts_per_turn(self.m)
        if mode == 1:
            lo, hi, unit = -1.0, 1.0, "duty -1..1"
        elif mode == 2:
            vmax = self.app.param_value(f"M{self.m + 1}_VEL_MAX") or 20000.0
            if scaled:
                vmax, unit = vmax / cpt * 60.0, "RPM (output)"
            else:
                unit = "counts/s"
            lo, hi = -vmax, vmax
        elif mode == 3:
            span, unit = (3600.0, "deg (output)") if scaled else (50000.0, "counts")
            lo, hi = -span, span
        else:
            lo, hi, unit = -1.0, 1.0, ""
        self.slider.configure(from_=lo, to=hi)
        self.slider_var.set(0.0)
        self.unit.configure(text=unit)

    def on_mode(self):
        self.app.cmd("set_mode", self.m, self.mode.get())
        self.target.set("0")
        self._set_slider_range()

    def on_send(self):
        try:
            v = float(self.target.get())
        except ValueError:
            self.app.status(f"Motor {self.m + 1}: target is not a number")
            return
        self.app.cmd("set_target", self.m, self.to_counts(v))

    def to_counts(self, v):
        """Target as shown (deg / RPM when scaled) -> what the board takes."""
        cpt, scaled = self.app.counts_per_turn(self.m)
        if scaled and self.mode.get() == 3:
            return v / 360.0 * cpt
        if scaled and self.mode.get() == 2:
            return v / 60.0 * cpt
        return v

    def on_slider(self, _v):
        if self.mode.get() == 0:
            return
        now = time.monotonic()
        if now - self._last_slider_send < 0.05:   # 20 updates/s max
            return
        self._last_slider_send = now
        v = self.slider_var.get()
        v = round(v, 3) if self.mode.get() == 1 else round(v)
        self.target.set(str(v))
        self.app.cmd("set_target", self.m, self.to_counts(v))

    def update_telem(self, t):
        self.vals["mode"].set(MODES[t["mode"]] if t["mode"] < 4 else "?")
        if self.app.counts_per_turn(self.m)[1]:
            self.vals["pos"].set(f"{t['pos_u']:.1f} deg")
            self.vals["vel"].set(f"{t['vel_u']:.1f} RPM")
        else:
            self.vals["pos"].set(f"{t['pos']}")
            self.vals["vel"].set(f"{t['vel']:.0f} cnt/s")
        self.vals["cur"].set("n/a (no IS)" if t["cur"] is None else f"{t['cur']:.3f} A")
        self.vals["duty"].set(f"{t['duty'] * 100:.1f} %")
        self.vals["faults"].set(fault_text(t["faults"]))
        self.fault_lbl.configure(foreground="#c53030" if t["faults"] else "")
        self.vals["limit"].set("PRESSED" if t["limit"] else "released")
        self.vals["home"].set("homing..." if t.get("homing") else "done" if t.get("homed") else "not homed")
        if t["mode"] != self.mode.get() and t["mode"] < 4:
            # the board changed mode on its own (fault, watchdog, STOP)
            self.mode.set(t["mode"])
            self._set_slider_range()


class Plot(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Label(bar, text="Plot:").pack(side="left")
        self.signal = tk.StringVar(value="Velocity")
        ttk.Combobox(bar, textvariable=self.signal, values=list(PLOT_SIGNALS),
                     state="readonly", width=18).pack(side="left", padx=4)
        for m in range(2):
            ttk.Label(bar, text=f"  Motor {m + 1}", foreground=MOTOR_COLORS[m]).pack(side="left")
        self.paused = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Pause", variable=self.paused).pack(side="right")
        self.canvas = tk.Canvas(self, height=220, background="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, pady=4)
        self.data = [collections.deque(maxlen=4000) for _ in range(2)]

    def add(self, m, t):
        self.data[m].append((time.monotonic(), t))

    def redraw(self):
        if self.paused.get():
            return
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or h < 50:
            return
        key = PLOT_SIGNALS[self.signal.get()]
        now = time.monotonic()
        series = []
        for m in range(2):
            pts = [(ts, s[key]) for ts, s in self.data[m]
                   if now - ts <= PLOT_SECONDS and s[key] is not None]
            series.append(pts)
        vals = [v for pts in series for _, v in pts]
        left, right, top, bottom = 75, w - 10, 10, h - 20
        if not vals:
            c.create_text(w / 2, h / 2, text="no data", fill="#888")
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pad = (hi - lo) * 0.1
        lo, hi = lo - pad, hi + pad

        def y(v):
            return bottom - (v - lo) / (hi - lo) * (bottom - top)

        def x(ts):
            return right - (now - ts) / PLOT_SECONDS * (right - left)

        for i in range(5):
            v = lo + (hi - lo) * i / 4
            c.create_line(left, y(v), right, y(v), fill="#eee")
            label = f"{v:.0f}" if abs(hi - lo) >= 20 else f"{v:.3f}"
            c.create_text(left - 4, y(v), text=label, anchor="e", fill="#666",
                          font=("TkDefaultFont", 8))
        if lo < 0 < hi:
            c.create_line(left, y(0), right, y(0), fill="#bbb")
        c.create_text(right, bottom + 10, text="now", anchor="e", fill="#666", font=("TkDefaultFont", 8))
        c.create_text(left, bottom + 10, text=f"-{PLOT_SECONDS:.0f} s", anchor="w", fill="#666",
                      font=("TkDefaultFont", 8))
        for m, pts in enumerate(series):
            if len(pts) >= 2:
                coords = [coord for ts, v in pts for coord in (x(ts), y(v))]
                c.create_line(*coords, fill=MOTOR_COLORS[m], width=2)


class ConfigTab(ttk.Frame):
    COLS = ("name", "device", "new", "unit", "range", "default", "desc")

    def __init__(self, master, app):
        super().__init__(master, padding=6)
        self.app = app
        bar = ttk.Frame(self)
        bar.pack(fill="x")
        for text, fn in [("Read all", self.read_all), ("Write changes", self.write_changes),
                         ("Save to flash", self.save), ("Restore defaults", self.defaults),
                         ("Export…", self.export), ("Import…", self.import_)]:
            ttk.Button(bar, text=text, command=fn).pack(side="left", padx=2)
        self.build = ttk.Label(bar, text="")
        self.build.pack(side="right")

        # The tree must be a child of `body` (not packed into it with in_=): a
        # sibling frame created after it stacks on top and hides it on Windows.
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, pady=4)
        self.tree = ttk.Treeview(body, columns=self.COLS, show="tree headings", height=18)
        heads = {"name": ("Parameter", 130), "device": ("On board", 90), "new": ("New value", 90),
                 "unit": ("Unit", 100), "range": ("Range", 110), "default": ("Default", 80),
                 "desc": ("Description", 330)}
        self.tree.column("#0", width=110)
        self.tree.heading("#0", text="Group")
        for col, (text, width) in heads.items():
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor="w")
        self.tree.tag_configure("changed", background="#fff3bf")
        self.tree.tag_configure("unused", foreground="#999")
        self.tree.tag_configure("error", background="#fed7d7")
        sb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.bind("<Double-1>", self.edit_cell)
        ttk.Label(self, text="Double-click a row to edit. Yellow = not yet written. "
                             "Write changes applies them now; Save to flash keeps them after power-off "
                             "(motors must be OFF).").pack(anchor="w")

        self.groups = {}
        self.rows = {}
        self.pending = {}
        for pid, name, group, unit, lo, hi, desc, _needs in PARAMS:
            if group not in self.groups:
                self.groups[group] = self.tree.insert("", "end", text=group, open=True)
            self.rows[pid] = self.tree.insert(self.groups[group], "end", iid=f"p{pid}", values=(
                name, "?", "", unit, f"{lo:g} .. {hi:g}", "", desc))

    def set_build(self, cs):
        for pid, p in BY_ID.items():
            needs = p[7]
            unused = (needs == "cs" and not cs) or (needs == "nocs" and cs)
            self.tree.item(self.rows[pid], tags=("unused",) if unused else ())

    def _set(self, pid, col, value):
        vals = list(self.tree.item(self.rows[pid], "values"))
        vals[self.COLS.index(col)] = value
        self.tree.item(self.rows[pid], values=vals)

    def on_param(self, pid, value, status):
        if pid not in self.rows:
            return
        if status:
            self.tree.item(self.rows[pid], tags=("error",))
            return
        self._set(pid, "device", f"{value:.6g}")
        if pid in self.pending and abs(self.pending[pid] - value) <= 1e-6 * max(1.0, abs(value)):
            del self.pending[pid]
            self._set(pid, "new", "")
            self.tree.item(self.rows[pid], tags=())

    def on_paramdef(self, pid, name, lo, hi, default):
        if pid not in self.rows:
            self.app.status(f"Board has parameter {pid} {name} the GUI doesn't know (update params.py)")
            return
        if BY_ID[pid][1] != name:
            self.app.status(f"Parameter {pid} is {name} on the board but {BY_ID[pid][1]} in the GUI")
        self._set(pid, "range", f"{lo:g} .. {hi:g}")
        self._set(pid, "default", f"{default:.6g}")

    def edit_cell(self, event):
        iid = self.tree.identify_row(event.y)
        if not iid.startswith("p"):
            return
        pid = int(iid[1:])
        x, y, w, h = self.tree.bbox(iid, "new")
        entry = ttk.Entry(self.tree)
        entry.place(x=x, y=y, width=w, height=h)
        entry.insert(0, self.tree.item(iid, "values")[2] or self.tree.item(iid, "values")[1])
        entry.select_range(0, "end")
        entry.focus()

        def commit(_e=None):
            text = entry.get().strip()
            entry.destroy()
            self.stage(pid, text)

        entry.bind("<Return>", commit)
        entry.bind("<FocusOut>", commit)
        entry.bind("<Escape>", lambda _e: entry.destroy())

    def stage(self, pid, text):
        if text in ("", "?"):
            return
        try:
            v = float(text)
        except ValueError:
            self.app.status(f"{BY_ID[pid][1]}: not a number")
            return
        lo, hi = BY_ID[pid][4], BY_ID[pid][5]
        if not lo <= v <= hi:
            self.app.status(f"{BY_ID[pid][1]} must be {lo:g} .. {hi:g}")
            return
        self.pending[pid] = v
        self._set(pid, "new", f"{v:.6g}")
        self.tree.item(self.rows[pid], tags=("changed",))

    def read_all(self):
        self.app.cmd("read_all")

    def write_changes(self):
        if not self.pending:
            self.app.status("Nothing to write")
            return
        for pid, v in sorted(self.pending.items()):
            self.app.cmd("write_param", pid, v)
            if self.app.link and self.app.link.kind == "CAN":
                time.sleep(0.005)
        self.app.status(f"Wrote {len(self.pending)} parameter(s)")

    def save(self):
        if self.pending and not messagebox.askyesno(
                "Unwritten changes", "Some edits are not written yet. Save what is on the board anyway?"):
            return
        self.app.cmd("save")

    def defaults(self):
        if messagebox.askyesno("Restore defaults",
                               "Reset all parameters on the board to the firmware defaults?\n"
                               "(Not saved to flash until you press Save to flash.)"):
            self.pending.clear()
            for pid in self.rows:
                self._set(pid, "new", "")
                self.tree.item(self.rows[pid], tags=())
            self.app.cmd("defaults")

    def export(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return
        out = {}
        for pid, iid in self.rows.items():
            dev = self.tree.item(iid, "values")[1]
            if dev != "?":
                out[BY_ID[pid][1]] = float(dev)
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
        self.app.status(f"Exported {len(out)} parameters to {path}")

    def import_(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path) as f:
            data = json.load(f)
        n = 0
        for name, v in data.items():
            if name in BY_NAME:
                self.stage(BY_NAME[name][0], str(v))
                n += 1
        self.app.status(f"Staged {n} parameters from {path}. Press Write changes to apply.")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual motor driver")
        self.geometry("1080x780")
        self.q = queue.Queue()
        self.link = None
        self.params = {}
        self._last_ping = 0.0

        self._build_connection_bar()
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=4)

        ctl = ttk.Frame(nb, padding=6)
        nb.add(ctl, text="Control")
        panels = ttk.Frame(ctl)
        panels.pack(fill="x")
        self.motors = [MotorPanel(panels, self, m) for m in range(2)]
        for m, p in enumerate(self.motors):
            p.grid(row=0, column=m, sticky="nsew", padx=4)
            panels.columnconfigure(m, weight=1)
        self.plot = Plot(ctl)
        self.plot.pack(fill="both", expand=True, pady=6)
        self.ppm = ttk.Label(ctl, text="PPM: - (reported over UART only)")
        self.ppm.pack(anchor="w")

        self.config_tab = ConfigTab(nb, self)
        nb.add(self.config_tab, text="Configuration")

        con = ttk.Frame(nb, padding=6)
        nb.add(con, text="Console")
        self.log = tk.Text(con, height=20, font=("TkFixedFont", 10), state="disabled")
        self.log.pack(fill="both", expand=True)
        row = ttk.Frame(con)
        row.pack(fill="x", pady=4)
        self.raw = tk.StringVar()
        e = ttk.Entry(row, textvariable=self.raw)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda _e: self.send_raw())
        ttk.Button(row, text="Send", command=self.send_raw).pack(side="left", padx=4)
        ttk.Label(con, text="UART: any protocol command (e.g. GET STAT).  CAN: id#hexdata or id#R for a request (e.g. 501#R = heartbeat of node 40).").pack(anchor="w")
        self.show_tx = tk.BooleanVar(value=False)
        ttk.Checkbutton(con, text="Show telemetry", variable=self.show_tx).pack(anchor="w")

        sbar = ttk.Frame(self, relief="sunken")
        sbar.pack(fill="x", side="bottom")
        self.statusbar = ttk.Label(sbar, text="Not connected", anchor="w", padding=3)
        self.statusbar.pack(side="left", fill="x", expand=True)
        self.info = ttk.Label(sbar, text="", anchor="e", padding=3)
        self.info.pack(side="right")

        self.bind_all("<Escape>", lambda _e: self.stop_all())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.pump)
        self.after(100, self.redraw)

    # ---------------------------------------------------------- connection
    def _build_connection_bar(self):
        bar = ttk.Frame(self, padding=6)
        bar.pack(fill="x")
        self.kind = tk.StringVar(value="UART")
        for k in ("UART", "CAN"):
            ttk.Radiobutton(bar, text=k, value=k, variable=self.kind,
                            command=self._show_link_opts).pack(side="left")

        self.uart_opts = ttk.Frame(bar)
        ttk.Label(self.uart_opts, text="Port").pack(side="left", padx=(8, 2))
        self.port = tk.StringVar()
        self.port_box = ttk.Combobox(self.uart_opts, textvariable=self.port, width=18)
        self.port_box.pack(side="left")
        ttk.Button(self.uart_opts, text="↻", width=2, command=self.refresh_ports).pack(side="left")
        ttk.Label(self.uart_opts, text="Baud").pack(side="left", padx=(8, 2))
        self.baud = tk.StringVar(value="115200")
        ttk.Entry(self.uart_opts, textvariable=self.baud, width=8).pack(side="left")

        self.can_opts = ttk.Frame(bar)
        ttk.Label(self.can_opts, text="Interface").pack(side="left", padx=(8, 2))
        self.can_if = tk.StringVar(value="slcan")
        ttk.Combobox(self.can_opts, textvariable=self.can_if, values=CAN_INTERFACES,
                     width=9).pack(side="left")
        ttk.Label(self.can_opts, text="Channel").pack(side="left", padx=(8, 2))
        self.can_ch = tk.StringVar()
        self.can_ch_box = ttk.Combobox(self.can_opts, textvariable=self.can_ch, width=14)
        self.can_ch_box.pack(side="left")
        ttk.Label(self.can_opts, text="Bitrate").pack(side="left", padx=(8, 2))
        self.can_br = tk.StringVar(value="125000")
        ttk.Entry(self.can_opts, textvariable=self.can_br, width=8).pack(side="left")
        ttk.Label(self.can_opts, text="Nodes M1/M2").pack(side="left", padx=(8, 2))
        self.can_nodes = tk.StringVar(value="40/41")
        ttk.Entry(self.can_opts, textvariable=self.can_nodes, width=6).pack(side="left")

        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.keepalive = tk.BooleanVar(value=True)
        self.keep_chk = ttk.Checkbutton(bar, text="Keep-alive", variable=self.keepalive)
        stop = tk.Button(bar, text="STOP ALL (Esc)", command=self.stop_all, bg="#c53030",
                         fg="white", activebackground="#9b2c2c", font=("TkDefaultFont", 10, "bold"))
        stop.pack(side="right")
        self._show_link_opts()
        self.refresh_ports()

    def _show_link_opts(self):
        for w in (self.uart_opts, self.can_opts, self.connect_btn, self.keep_chk):
            w.pack_forget()
        (self.uart_opts if self.kind.get() == "UART" else self.can_opts).pack(side="left")
        self.connect_btn.pack(side="left", padx=8)
        self.keep_chk.pack(side="left")

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port_box.configure(values=ports)
        self.can_ch_box.configure(values=ports + ["can0", "PCAN_USBBUS1"])
        if ports and not self.port.get():
            self.port.set(ports[0])

    def toggle_connect(self):
        if self.link:
            self.disconnect("Disconnected")
            return
        try:
            if self.kind.get() == "UART":
                self.link = UartLink(self.q, self.port.get(), int(self.baud.get()))
                where = self.port.get()
            else:
                n1, n2 = (int(x, 0) for x in self.can_nodes.get().split("/"))
                self.link = CanLink(self.q, self.can_if.get(), self.can_ch.get(),
                                    int(self.can_br.get()), (n1, n2))
                where = f"{self.can_if.get()} {self.can_ch.get()}"
        except Exception as e:
            self.link = None
            messagebox.showerror("Connect failed", str(e))
            return
        self.connect_btn.configure(text="Disconnect")
        self._where = where
        self._answered = False
        self._connect_t = self._last_info = time.monotonic()
        self._warned = False
        self.status(f"Port {where} open, waiting for the board to answer...")
        self.link.info()
        self.after(150, lambda: self.cmd("read_all"))

    def disconnect(self, why):
        if self.link:
            self.link.close()
        self.link = None
        self.connect_btn.configure(text="Connect")
        self.info.configure(text="")
        self.status(why)

    def on_close(self):
        if self.link:
            self.link.close()
        self.destroy()

    # ------------------------------------------------------------- commands
    def cmd(self, name, *args):
        if not self.link:
            self.status("Not connected")
            return
        try:
            getattr(self.link, name)(*args)
        except Exception as e:
            self.status(f"{name} failed: {e}")

    def stop_all(self):
        self.cmd("stop")

    def send_raw(self):
        text = self.raw.get().strip()
        if text:
            self.cmd("raw", text)
            self.raw.set("")

    def counts_per_turn(self, m):
        """(counts per output turn, True when that makes deg/RPM meaningful)."""
        cpt = counts_per_turn(self.params, m)
        return cpt, cpt > 0 and abs(cpt - 1.0) > 1e-6

    def param_value(self, name):
        p = BY_NAME.get(name)
        return self.params.get(p[0]) if p else None

    def status(self, text):
        self.statusbar.configure(text=time.strftime("%H:%M:%S  ") + text)

    def logline(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 2000:
            self.log.delete("1.0", "500.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ------------------------------------------------------------ event pump
    def pump(self):
        try:
            for _ in range(500):
                ev = self.q.get_nowait()
                kind = ev[0]
                if self.link and not self._answered and kind in ("telem", "param", "paramdef", "info"):
                    self._answered = True
                    self.status(f"Connected via {self.link.kind} ({self._where}), board answered")
                if kind == "telem":
                    _, m, t = ev
                    cpt, scaled = self.counts_per_turn(m)
                    t["pos_u"] = t["pos"] / cpt * 360.0 if scaled else t["pos"]
                    t["vel_u"] = t["vel"] / cpt * 60.0 if scaled else t["vel"]
                    self.motors[m].update_telem(t)
                    self.plot.add(m, t)
                    if self.show_tx.get():
                        self.logline(f"< M{m + 1} {t}")
                elif kind == "param":
                    _, pid, val, st = ev
                    if not st:
                        changed = self.params.get(pid) != val
                        self.params[pid] = val
                        if changed and pid in UNIT_PARAMS | {BY_NAME["M1_VEL_MAX"][0], BY_NAME["M2_VEL_MAX"][0]}:
                            for panel in self.motors:
                                panel._set_slider_range()
                    self.config_tab.on_param(pid, val, st)
                elif kind == "paramdef":
                    _, pid, name, _val, lo, hi, de = ev
                    self.config_tab.on_paramdef(pid, name, lo, hi, de)
                elif kind == "info":
                    i = ev[1]
                    self.info.configure(text=f"FW {i['fw']}  ENC1={i['enc1']}  "
                                             f"current sense {'on' if i['cs'] else 'off'}")
                    self.config_tab.set_build(i["cs"])
                    if i["params"] != len(PARAMS):
                        self.status(f"Board has {i['params']} parameters, GUI knows {len(PARAMS)}")
                elif kind == "ppm":
                    self.ppm.configure(text=f"PPM1: {ev[1] or '-'} µs   PPM2: {ev[2] or '-'} µs")
                elif kind == "event":
                    self.status(ev[1])
                    self.logline("! " + ev[1])
                elif kind == "log":
                    self.logline(ev[1])
                elif kind == "closed":
                    self.disconnect(f"Link closed: {ev[1]}")
        except queue.Empty:
            pass
        now = time.monotonic()
        if self.link and not self._answered and now - self._last_info >= 1.0:
            self._last_info = now
            try:
                self.link.info()
            except Exception as e:
                self.disconnect(f"Link error: {e}")
            if self.link and not self._warned and now - self._connect_t >= 3.0:
                self._warned = True
                self.status(f"No reply from the board on {self._where}. Check the board has power "
                            "and the wiring is crossed (adapter TX -> PB11, RX -> PB10, GND).")
        if self.link and self.keepalive.get() and now - self._last_ping >= KEEPALIVE_S:
            self._last_ping = now
            try:
                self.link.ping()
            except Exception as e:
                self.disconnect(f"Link error: {e}")
        self.after(30, self.pump)

    def redraw(self):
        self.plot.redraw()
        self.after(100, self.redraw)


if __name__ == "__main__":
    App().mainloop()
