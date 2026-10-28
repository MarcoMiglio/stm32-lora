"""PyInstaller runtime hook for bundled Tcl/Tk on Windows.

Runs before network_dashboard.py. The build script places Tcl/Tk DLLs in
sys._MEIPASS and their script libraries in _tcl_data / _tk_data.
"""
import builtins
import os
import sys
from pathlib import Path

if sys.platform == "win32" and getattr(sys, "frozen", False):
    bundle_dir = Path(sys._MEIPASS).resolve()

    # Point Tcl/Tk at the script libraries bundled by build_portable.py.
    tcl_dir = bundle_dir / "_tcl_data"
    tk_dir = bundle_dir / "_tk_data"
    if tcl_dir.is_dir():
        os.environ["TCL_LIBRARY"] = str(tcl_dir)
    if tk_dir.is_dir():
        os.environ["TK_LIBRARY"] = str(tk_dir)

    # Explicitly add the bundle directory to the Windows DLL search path.
    # Keep the returned handle alive for the lifetime of the process.
    if hasattr(os, "add_dll_directory"):
        try:
            builtins.__LORA_TK_DLL_DIR_HANDLE = os.add_dll_directory(str(bundle_dir))
        except OSError:
            pass

    os.environ["PATH"] = str(bundle_dir) + os.pathsep + os.environ.get("PATH", "")
