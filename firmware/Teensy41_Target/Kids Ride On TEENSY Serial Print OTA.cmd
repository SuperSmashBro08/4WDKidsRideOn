@echo off
echo ===============================
echo  KIDS RIDE ON TELNET CONSOLE LAUNCHER
echo ===============================
echo.

set HOST=192.168.1.29
set PORT=2323

echo Connecting to %HOST% on port %PORT% ...
echo (Press Ctrl+] then type 'quit' to exit Telnet)
echo.

REM Simple ping check
ping -n 1 %HOST% >nul
if errorlevel 1 (
    echo ❌ Could not reach %HOST%.
    echo Make sure the ESP32 is powered on and connected to WiFi.
    pause
    exit /b
)

REM Start the Windows Telnet client
start telnet %HOST% %PORT%
