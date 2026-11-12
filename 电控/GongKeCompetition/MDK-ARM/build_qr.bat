@echo off
cd /d "C:\Users\Administrator\Desktop\GongKeCompetition\MDK-ARM"
"C:\Users\Administrator\AppData\Local\Keil_v5\UV4\UV4.exe" -b GongKeCompetition.uvprojx -j0 -o build_qr.log
echo BUILD_EXIT=%ERRORLEVEL%
