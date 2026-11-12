@echo off
rem ============================================================
rem  Stop the label/verify platform.
rem  Kills whatever process is LISTENING on port 8765 (/T = whole
rem  process tree), so there is no orphan python left behind.
rem  Content kept ASCII-only (see 启动标注平台.bat for why).
rem ============================================================
title LineLabel :8765 -- stopping
setlocal

set SRVPID=
for /f "tokens=5" %%p in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":8765 "') do set SRVPID=%%p
if not defined SRVPID goto noproc

echo.
echo  Found a listener on port 8765, PID=%SRVPID%
tasklist /FI "PID eq %SRVPID%"
echo.
choice /C YN /M "Kill PID %SRVPID% (and its children)"
if errorlevel 2 goto cancel

taskkill /PID %SRVPID% /T /F
echo.
echo  Verifying the port is free...
ping -n 2 127.0.0.1 >nul
netstat -ano | findstr "LISTENING" | findstr ":8765 " >nul
if errorlevel 1 goto freed
echo  STILL listening -- something else took the port; check Task Manager.
goto done

:freed
echo  Port 8765 is free. Server stopped.

:done
pause
exit /b 0

:noproc
echo.
echo  Nothing is listening on port 8765 -- no server to stop.
pause
exit /b 0

:cancel
echo.
echo  Cancelled. Nothing was killed.
pause
exit /b 0
