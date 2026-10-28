# LoRa Network Dashboard V1.10 - Windows portable build

This version fixes the Tcl/Tk packaging problem seen when building the Tkinter GUI from Miniconda.

## Why V1.8 could fail

The source application can use Tkinter correctly from Miniconda, but `_tkinter.pyd` depends on native Tcl/Tk DLLs that Conda normally stores under `Library\\bin`. A PyInstaller build launched by calling `python.exe` directly may fail to resolve/copy those libraries. The resulting executable then raises `ImportError: DLL load failed while importing _tkinter`.

## What V1.10 does differently

`build_portable.py`:

- queries the working Tkinter installation for the exact Tcl and Tk script directories;
- locates the matching Tcl/Tk DLLs in Miniconda;
- adds Miniconda `Library\\bin` to the build-time DLL search path;
- explicitly bundles the Tcl/Tk DLLs and script libraries;
- installs a small PyInstaller runtime hook that sets `TCL_LIBRARY`, `TK_LIBRARY`, and the DLL search directory before `tkinter` is imported;
- checks that the important Tk assets are actually present in `dist` before producing the portable ZIP.

## Build

On the development PC, double-click:

`build_windows.bat`

It uses `%USERPROFILE%\\miniconda3\\python.exe`, updates PyInstaller 6.x, and creates:

- `dist\\LoRaDashboard\\LoRaDashboard.exe`
- `LoRaDashboard_portable.zip`

## Test

First launch `test_built_app.bat` on the development PC. Check that the GUI opens and that serial acquisition works.

Then extract `LoRaDashboard_portable.zip` on a second Windows PC that does not have Python installed and run `LoRaDashboard.exe`.

Do not copy only the EXE: the complete `LoRaDashboard` directory is required for the onedir build.

## V1.10 fix

Miniconda Python 3.12 can return `tk_library` as a `Tcl_Obj` rather than a native Python `str`. The build helper now explicitly converts Tcl/Tk interpreter values to strings before passing them to `pathlib.Path`. This fixes the `TypeError: argument should be a str or an os.PathLike object ... not Tcl_Obj` seen in V1.9.
