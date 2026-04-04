@echo off
:: Auto-elevate to admin
net session >nul 2>&1
if %errorlevel% neq 0 (
    powershell -Command "Start-Process '%~f0' -Verb RunAs"
    exit /b
)

title Re-enabling Wi-Fi
echo Enabling network adapters...

netsh interface set interface "Wi-Fi"   ENABLED 2>nul
netsh interface set interface "Wi-Fi 2" ENABLED 2>nul

powershell -NoProfile -Command ^
  "Get-NetAdapter | Where-Object { $_.InterfaceDescription -like '*Wireless*' -or $_.Name -like '*Wi-Fi*' } | Enable-NetAdapter -Confirm:$false" 2>nul

echo Enabling firewall...
netsh advfirewall set allprofiles state on >nul 2>&1
