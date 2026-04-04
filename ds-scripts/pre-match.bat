@echo off
:: Auto-elevate to admin
net session >nul 2>&1
if %errorlevel% neq 0 (
    powershell -Command "Start-Process '%~f0' -Verb RunAs"
    exit /b
)

title FRC Match Prep
echo [1/4] Closing all windowed applications...

powershell -NoProfile -Command ^
  "Get-Process | Where-Object { $_.MainWindowHandle -ne 0 -and $_.Name -ne 'explorer' -and $_.Name -ne 'cmd' } | Stop-Process -Force" 2>nul

echo [2/4] Disabling firewall...
netsh advfirewall set allprofiles state off >nul 2>&1

echo [3/4] Disabling network adapters...
netsh interface set interface "Wi-Fi"   DISABLED 2>nul
netsh interface set interface "Wi-Fi 2" DISABLED 2>nul

powershell -NoProfile -Command ^
  "Get-NetAdapter | Where-Object { $_.InterfaceDescription -like '*Wireless*' -or $_.Name -like '*Wi-Fi*' } | Disable-NetAdapter -Confirm:$false" 2>nul

echo [4/4] Waiting for services to settle...
timeout /t 2 /nobreak >nul

echo Launching Driver Station...
start "" /max "C:\Program Files (x86)\FRC Driver Station\DriverStation.exe"

echo Done. Good luck!