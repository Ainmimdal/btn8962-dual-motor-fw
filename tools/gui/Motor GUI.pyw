"""Double-click launcher for the motor GUI (Windows: opens with no console).

Installs pyserial / python-can the first time if they are missing, then starts
motor_gui.py from this folder.
"""
import importlib.util
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)

missing = [m for m in ("serial", "can") if importlib.util.find_spec(m) is None]
if missing:
    python = sys.executable
    if python.lower().endswith("pythonw.exe"):
        python = python[:-len("pythonw.exe")] + "python.exe"
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run([python, "-m", "pip", "install", "-r", "requirements.txt"],
                            capture_output=True, text=True, creationflags=flags)
    if result.returncode != 0:
        import tkinter as tk
        from tkinter import messagebox
        tk.Tk().withdraw()
        messagebox.showerror("Motor GUI", "Could not install pyserial / python-can:\n\n"
                             + (result.stderr or result.stdout)[-1500:])
        sys.exit(1)

import runpy  # noqa: E402

runpy.run_path(os.path.join(HERE, "motor_gui.py"), run_name="__main__")
