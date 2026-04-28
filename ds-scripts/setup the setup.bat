@echo off
setlocal

set "DESKTOP=%USERPROFILE%\Desktop"

echo ====================================
echo Installing FRC Match Prep Tools...
echo ====================================

:: -----------------------------
:: CREATE MATCH PREP SCRIPT
:: -----------------------------
(
echo @echo off
echo net session ^>nul 2^>^&1
echo if %%errorlevel%% neq 0 ^(
echo     powershell -Command "Start-Process '%%~f0' -Verb RunAs"
echo     exit /b
echo ^)
echo title FRC Match Prep
echo echo [1/4] Closing apps...
echo powershell -NoProfile -Command "Get-Process ^| Where-Object { $_.MainWindowHandle -ne 0 -and $_.Name -ne 'explorer' -and $_.Name -ne 'cmd' } ^| Stop-Process -Force" 2^>nul
echo echo [2/4] Disabling firewall...
echo netsh advfirewall set allprofiles state off ^>nul 2^>^&1
echo echo [3/4] Disabling Wi-Fi...
echo netsh interface set interface "Wi-Fi" DISABLED 2^>nul
echo powershell -NoProfile -Command "Get-NetAdapter ^| Where-Object { $_.Name -like '*Wi-Fi*' } ^| Disable-NetAdapter -Confirm:$false" 2^>nul
echo echo Disabling Bluetooth...
echo powershell -NoProfile -Command "Get-NetAdapter ^| Where-Object { $_.Name -like '*Bluetooth*' } ^| Disable-NetAdapter -Confirm:$false" 2^>nul
echo timeout /t 2 /nobreak ^>nul
echo start "" /max "C:\Program Files (x86)\FRC Driver Station\DriverStation.exe"
echo echo Done. Good luck!
) > "%DESKTOP%\FRC_Match_Prep.bat"

:: -----------------------------
:: CREATE RESTORE SCRIPT
:: -----------------------------
(
echo @echo off
echo net session ^>nul 2^>^&1
echo if %%errorlevel%% neq 0 ^(
echo     powershell -Command "Start-Process '%%~f0' -Verb RunAs"
echo     exit /b
echo ^)
echo title FRC Restore Network
echo echo Enabling Wi-Fi...
echo netsh interface set interface "Wi-Fi" ENABLED 2^>nul
echo powershell -NoProfile -Command "Get-NetAdapter ^| Where-Object { $_.Name -like '*Wi-Fi*' } ^| Enable-NetAdapter -Confirm:$false" 2^>nul
echo echo Enabling Bluetooth...
echo powershell -NoProfile -Command "Get-NetAdapter ^| Where-Object { $_.Name -like '*Bluetooth*' } ^| Enable-NetAdapter -Confirm:$false" 2^>nul
echo echo Enabling firewall...
echo netsh advfirewall set allprofiles state on ^>nul 2^>^&1
echo echo Done.
) > "%DESKTOP%\FRC_Restore_Network.bat"

:: -----------------------------
:: CREATE SHORTCUTS WITH ICONS
:: -----------------------------
echo Creating shortcuts...

powershell -NoProfile -Command ^
"$ws = New-Object -ComObject WScript.Shell; ^
$desktop = [Environment]::GetFolderPath('Desktop'); ^
$system32 = [Environment]::GetFolderPath('System'); ^

$prep = $ws.CreateShortcut($desktop + '\Butterfly.lnk'); ^
$prep.TargetPath = $system32 + '\cmd.exe'; ^
$prep.Arguments = '/c \"\"' + $desktop + '\FRC_Match_Prep.bat\"\"'; ^
$prep.IconLocation = 'shell32.dll,238'; ^
$prep.WindowStyle = 1; ^
$prep.Save(); ^

$restore = $ws.CreateShortcut($desktop + '\Dial Up Internet.lnk'); ^
$restore.TargetPath = $system32 + '\cmd.exe'; ^
$restore.Arguments = '/c \"\"' + $desktop + '\FRC_Restore_Network.bat\"\"'; ^
$restore.IconLocation = 'shell32.dll,250'; ^
$restore.WindowStyle = 1; ^
$restore.Save();"

echo.
echo ====================================
echo Installation complete!
echo ====================================
echo.
echo Desktop shortcuts created:
echo - Butterfly (Match Prep)
echo - Dial Up Internet (Restore Network)
echo.
pause
