@echo off
rem ================================================================
rem  已知内容候选集配置:
rem    123      = 三位数, 每位取 1/2/3, 共 27 种 (111~333)
rem    12       = 三位数, 每位取 1/2,  共 8 种
rem    0123:4   = 四位数, 每位取 0~9
rem    111,222  = 显式列表
rem    留空     = 通用模式 (较慢, 自动回退)
set "KNOWN_DEFAULT=123"
rem ================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"
title DiffcultQR 解码速度测试

:again
echo.
echo ==================================================
echo   DiffcultQR 二维码解码速度测试
echo   把二维码图片拖进本窗口并按回车即可测试
echo   多个文件只测第一个 - 输入q退出
echo ==================================================
echo.
set "IMG="
set /p "IMG=图片路径: "
if not defined IMG goto end
if /i "!IMG!"=="q" goto end

set "FIRST="
for %%F in (!IMG!) do if not defined FIRST set "FIRST=%%~fF"
if not defined FIRST goto again

if not exist "!FIRST!" (
    echo [错误] 文件不存在: !FIRST!
    pause
    goto again
)

set "KNOWN="
if defined KNOWN_DEFAULT set "KNOWN=!KNOWN_DEFAULT!"
if not defined KNOWN (
    set /p "KNOWN=已知内容集合, 直接回车=通用模式, 示例 123 或 111,222 : "
)

echo.
echo 目标: !FIRST!
if defined KNOWN echo 候选集: !KNOWN!
echo --------------------------------------------------
if defined KNOWN (
    python -X utf8 -c "import time,subprocess,sys,os;env=dict(os.environ);env['PYTHONIOENCODING']='utf-8';t0=time.perf_counter();r=subprocess.run([sys.executable,'decode_qr.py','--known',sys.argv[1],sys.argv[2]],capture_output=True,text=True,encoding='utf-8',errors='replace',env=env);dt=time.perf_counter()-t0;print(r.stdout,end='');print(r.stderr,end='');print('-'*46);print(f'[计时] 端到端(含Python启动): {dt:.2f} 秒');sys.exit(r.returncode)" "!KNOWN!" "!FIRST!"
) else (
    python -X utf8 -c "import time,subprocess,sys,os;env=dict(os.environ);env['PYTHONIOENCODING']='utf-8';t0=time.perf_counter();r=subprocess.run([sys.executable,'decode_qr.py',sys.argv[1]],capture_output=True,text=True,encoding='utf-8',errors='replace',env=env);dt=time.perf_counter()-t0;print(r.stdout,end='');print(r.stderr,end='');print('-'*46);print(f'[计时] 端到端(含Python启动): {dt:.2f} 秒');sys.exit(r.returncode)" "!FIRST!"
)
echo.
pause
goto again

:end
endlocal
exit /b 0
