LoRa Network Dashboard - Portable Windows build
================================================

HOW TO RUN ON A TARGET PC

1. Copy/extract the ENTIRE LoRaDashboard folder to a writable location,
   for example Desktop, Documents, or a laboratory data folder.
2. Double-click LoRaDashboard.exe.
3. Connect the ROOT device by USB/serial.
4. Select its COM port in the dashboard and press Connect.

Python, Miniconda, pip, Tkinter and pyserial are NOT required on the target PC.
They are bundled into this portable folder by PyInstaller.

IMPORTANT
- Do not copy only LoRaDashboard.exe. Keep all files/subfolders generated
  by PyInstaller together.
- The application writes dashboard_settings.json and the default logs folder
  next to LoRaDashboard.exe. Therefore place the portable folder somewhere
  the current Windows user can write to. Avoid C:\Program Files for this
  portable version.
- A serial/USB driver may still be required if Windows does not expose the
  ROOT device as a COM port. This is independent from Python/the dashboard.
- The executable is not digitally signed. Windows SmartScreen can therefore
  show a warning on some PCs. Only run builds you created/trust.

DATA
- Default CSV folder: .\logs\ next to LoRaDashboard.exe
- The CSV folder can also be selected manually from the dashboard.
- dashboard_settings.json is created next to the executable after settings
  are saved.
