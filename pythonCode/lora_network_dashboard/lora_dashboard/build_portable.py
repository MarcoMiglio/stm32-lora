from __future__ import annotations

import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

APP_NAME = "LoRaDashboard"
HERE = Path(__file__).resolve().parent
SOURCE = HERE / "network_dashboard.py"
RUNTIME_HOOK = HERE / "runtime_tk_fix.py"
README_PORTABLE = HERE / "README_PORTABLE.txt"


def fail(message: str) -> None:
    print(f"\nERROR: {message}")
    raise SystemExit(1)


def find_dll(prefix: Path, names: list[str]) -> Path:
    search_dirs = [
        prefix / "Library" / "bin",   # Conda/Miniconda on Windows
        prefix / "DLLs",               # python.org Windows layout
        prefix,
    ]
    for directory in search_dirs:
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate.resolve()
    fail(
        "Could not find required Tcl/Tk DLL. Searched:\n  "
        + "\n  ".join(str(p) for p in search_dirs)
        + "\nExpected one of: " + ", ".join(names)
    )
    raise AssertionError


def detect_tcl_tk() -> tuple[Path, Path, Path, Path, Path | None]:
    import _tkinter
    import tkinter as tk

    prefix = Path(sys.prefix).resolve()

    # Tcl library path comes directly from the interpreter that we know works.
    interp = tk.Tcl()
    tcl_dir = Path(str(interp.eval("info library"))).resolve()

    # Ask an actual Tk instance for its script-library path. This is more
    # reliable than guessing the layout of a Conda distribution.
    root = tk.Tk()
    root.withdraw()
    try:
        tk_dir = Path(str(root.tk.getvar("tk_library"))).resolve()
    finally:
        root.destroy()

    if not (tcl_dir / "init.tcl").is_file():
        fail(f"Tcl library is not usable: {tcl_dir} (init.tcl missing)")
    if not tk_dir.is_dir():
        fail(f"Tk library directory does not exist: {tk_dir}")

    # 8.6 -> 86; support both threaded ('t') and non-threaded DLL names.
    tcl_digits = "".join(str(_tkinter.TCL_VERSION).split(".")[:2])
    tk_digits = "".join(str(_tkinter.TK_VERSION).split(".")[:2])

    tcl_dll = find_dll(prefix, [f"tcl{tcl_digits}t.dll", f"tcl{tcl_digits}.dll"])
    tk_dll = find_dll(prefix, [f"tk{tk_digits}t.dll", f"tk{tk_digits}.dll"])

    # Optional Tcl modules directory (PyInstaller normally collects this too).
    tcl_module_dir = tcl_dir.parent / f"tcl{str(_tkinter.TCL_VERSION).split('.')[0]}"
    if not tcl_module_dir.is_dir():
        tcl_module_dir = None

    print("\nDetected Tcl/Tk installation:")
    print(f"  Python prefix : {prefix}")
    print(f"  _tkinter      : {_tkinter.__file__}")
    print(f"  Tcl version   : {_tkinter.TCL_VERSION}")
    print(f"  Tk version    : {_tkinter.TK_VERSION}")
    print(f"  Tcl scripts   : {tcl_dir}")
    print(f"  Tk scripts    : {tk_dir}")
    print(f"  Tcl DLL       : {tcl_dll}")
    print(f"  Tk DLL        : {tk_dll}")
    print(f"  Tcl modules   : {tcl_module_dir or '(not present)'}")

    return tcl_dir, tk_dir, tcl_dll, tk_dll, tcl_module_dir


def run(cmd: list[str], env: dict[str, str]) -> None:
    print("\n> " + subprocess.list2cmdline(cmd))
    result = subprocess.run(cmd, cwd=HERE, env=env)
    if result.returncode != 0:
        raise SystemExit(result.returncode)


def main() -> None:
    if sys.platform != "win32":
        fail("This build helper is intended for Windows.")
    if not SOURCE.is_file():
        fail(f"Missing source file: {SOURCE}")

    # Verify application imports in the same interpreter used for packaging.
    import serial
    import tkinter
    print(f"Python      : {sys.executable}")
    print(f"Python ver. : {sys.version.split()[0]}")
    print(f"Tkinter     : {tkinter.TkVersion}")
    print(f"pyserial    : {serial.__version__}")

    tcl_dir, tk_dir, tcl_dll, tk_dll, tcl_module_dir = detect_tcl_tk()
    prefix = Path(sys.prefix).resolve()

    env = os.environ.copy()
    env["TCL_LIBRARY"] = str(tcl_dir)
    env["TK_LIBRARY"] = str(tk_dir)

    # Critical for Conda: allow PyInstaller's binary dependency scanner to
    # resolve the DLLs linked by _tkinter.pyd and their dependencies.
    dll_search_dirs = [prefix / "Library" / "bin", prefix / "DLLs", prefix]
    env["PATH"] = os.pathsep.join(str(p) for p in dll_search_dirs if p.is_dir()) + os.pathsep + env.get("PATH", "")

    for name in ("build", "dist"):
        path = HERE / name
        if path.exists():
            shutil.rmtree(path)
    spec = HERE / f"{APP_NAME}.spec"
    if spec.exists():
        spec.unlink()
    portable_zip = HERE / f"{APP_NAME}_portable.zip"
    if portable_zip.exists():
        portable_zip.unlink()

    sep = os.pathsep  # ';' on Windows for PyInstaller --add-data/--add-binary
    cmd = [
        sys.executable,
        "-m", "PyInstaller",
        "--noconfirm",
        "--clean",
        "--windowed",
        "--onedir",
        "--name", APP_NAME,
        "--hidden-import", "serial.tools.list_ports_windows",
        "--runtime-hook", str(RUNTIME_HOOK),
        # Explicit collection makes the bundle independent of Conda's DLL
        # search configuration on the target computer.
        "--add-binary", f"{tcl_dll}{sep}.",
        "--add-binary", f"{tk_dll}{sep}.",
        "--add-data", f"{tcl_dir}{sep}_tcl_data",
        "--add-data", f"{tk_dir}{sep}_tk_data",
    ]
    if tcl_module_dir is not None:
        cmd += ["--add-data", f"{tcl_module_dir}{sep}{tcl_module_dir.name}"]
    cmd.append(str(SOURCE))

    run(cmd, env)

    dist_dir = HERE / "dist" / APP_NAME
    exe = dist_dir / f"{APP_NAME}.exe"
    if not exe.is_file():
        fail(f"PyInstaller completed but executable was not found: {exe}")

    # Basic bundle integrity checks before handing it to the user.
    internal = dist_dir / "_internal"
    required_candidates = {
        "_tkinter extension": list(internal.rglob("_tkinter*.pyd")),
        "Tcl DLL": list(internal.rglob(tcl_dll.name)),
        "Tk DLL": list(internal.rglob(tk_dll.name)),
        "Tcl init.tcl": list(internal.rglob("_tcl_data/init.tcl")),
    }
    missing = [label for label, paths in required_candidates.items() if not paths]
    if missing:
        print("\nWARNING: build completed, but these expected Tk assets were not found:")
        for label in missing:
            print(f"  - {label}")
        print("Do not distribute the build until it has been tested.")
    else:
        print("\nTk bundle integrity check: OK")
        for label, paths in required_candidates.items():
            print(f"  {label}: {paths[0].relative_to(dist_dir)}")

    (dist_dir / "logs").mkdir(exist_ok=True)
    if README_PORTABLE.is_file():
        shutil.copy2(README_PORTABLE, dist_dir / "README.txt")

    with zipfile.ZipFile(portable_zip, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in dist_dir.rglob("*"):
            if path.is_file():
                zf.write(path, Path(APP_NAME) / path.relative_to(dist_dir))

    print("\n========================================")
    print("BUILD COMPLETE")
    print("========================================")
    print(f"Test executable:\n  {exe}")
    print(f"Portable ZIP:\n  {portable_zip}")
    print("\nIMPORTANT: test the EXE on this PC before copying it elsewhere.")


if __name__ == "__main__":
    main()
