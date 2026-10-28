# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['C:/Users/LabMeas/Desktop/MarcoMiglio/work_lora/stm32-lora/pythonCode/lora_network_dashboard_v1_10_tk_fix/lora_dashboard_v1_10_packaging_fix/network_dashboard.py'],
    pathex=[],
    binaries=[('C:/Users/LabMeas/miniconda3/Library/bin/tcl86t.dll', '.'), ('C:/Users/LabMeas/miniconda3/Library/bin/tk86t.dll', '.')],
    datas=[('C:/Users/LabMeas/miniconda3/Library/lib/tcl8.6', '_tcl_data'), ('C:/Users/LabMeas/miniconda3/Library/lib/tk8.6', '_tk_data'), ('C:/Users/LabMeas/miniconda3/Library/lib/tcl8', 'tcl8')],
    hiddenimports=['serial.tools.list_ports_windows'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=['C:/Users/LabMeas/Desktop/MarcoMiglio/work_lora/stm32-lora/pythonCode/lora_network_dashboard_v1_10_tk_fix/lora_dashboard_v1_10_packaging_fix/runtime_tk_fix.py'],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='LoRaDashboard',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='LoRaDashboard',
)
