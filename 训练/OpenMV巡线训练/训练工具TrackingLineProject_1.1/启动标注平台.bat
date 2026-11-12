@echo off
rem ============================================================
rem  Tracking-line label/verify platform : one-click launcher
rem  Double-click me. THIS window owns the server process:
rem    close this window  ==  stop the server (clean kill)
rem  The PID is printed below and by server.py itself.
rem  Keep the content ASCII-only: a UTF-8 Chinese echo would
rem  come out as mojibake under the default 936 codepage.
rem ============================================================
title LineLabel :8765 -- close me to STOP the server
cd /d "%~dp0labeltool"

rem ---- already running? tell the user WHICH process owns the port ----
set RUNPID=
for /f "tokens=5" %%p in ('netstat -ano ^| findstr "LISTENING" ^| findstr ":8765 "') do set RUNPID=%%p
if not defined RUNPID goto startserver

echo.
echo  A server is ALREADY listening on port 8765, PID=%RUNPID%
echo.
echo  This window is only the launcher and will close now.
echo  That process belongs to a window/detached run, not this one.
echo    - stop it cleanly : taskkill /PID %RUNPID% /T
echo    - or double-click  the stop script next to this file
echo.
start "" http://127.0.0.1:8765
pause
exit /b 0

:startserver
echo.
echo  Starting platform on http://127.0.0.1:8765
echo  CLOSE THIS WINDOW to stop it (clean kill of this process tree).
echo  After editing main.py you do NOT need to restart:
echo    verify tab -^> R  (reloads main.py and re-runs the current frame)
echo.
rem ---- open browser after ~2s, without blocking python ----
start "" /b cmd /c "ping -n 3 127.0.0.1 >nul & start http://127.0.0.1:8765"
python server.py

echo.
echo  Server exited. If there was an error above: check that python is in PATH.
pause
