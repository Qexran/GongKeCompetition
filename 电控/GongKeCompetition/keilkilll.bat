@echo off
rem ============================================================
rem  Keil 工程清理脚本（加固版）
rem  - 放在工程目录或其上级目录运行，只清理脚本所在目录树
rem  - 按精确扩展名匹配，规避 8.3 短文件名误伤：
rem      del *.htm /s 会命中 .html（实测删掉过 webconsole 的 index.html）
rem      del *.bak /s 会命中 .bak-xxx（实测命中 113 个 .bak-<主题> 手术备份）
rem  - 刻意保护、绝不删除：
rem      *.hex 固件 / *.uvprojx / *.uvoptx(JLink与调试设置) / DebugConfig /
rem      RTE(RTE_Components.h) / JLinkSettings.ini / *.bak-<主题> 手术备份 /
rem      *.html / 各类非 build*.log 的日志
rem ============================================================
cd /d "%~dp0"

rem ==== 防呆：本目录或一级子目录必须找得到 Keil 工程，否则不执行任何删除 ====
set KEILFOUND=0
if exist *.uvprojx set KEILFOUND=1
if exist *.uvproj set KEILFOUND=1
if %KEILFOUND%==0 for /d %%d in (*) do if exist "%%d\*.uvprojx" set KEILFOUND=1
if %KEILFOUND%==0 for /d %%d in (*) do if exist "%%d\*.uvproj" set KEILFOUND=1
if %KEILFOUND%==0 (
    echo [中止] 本目录及一级子目录未找到 Keil 工程，为防误删已退出。
    pause
    exit /b 1
)

rem ==== 构建中间产物：精确扩展名匹配（del /s 的通配会经 8.3 短名误伤别的类型）====
for %%e in (bak ddk edk lst lnp mpf mpj obj omf plg rpt tmp __i crf o d axf tra dep iex htm sct map) do (
    for /r %%f in (*.%%e) do if /i "%%~xf"==".%%e" del /q "%%f"
)
rem ==== UV4 构建日志 build*.log（全树只清这一类日志，其余日志一律不碰）====
for /r %%f in (build*.log) do if /i "%%~xf"==".log" del /q "%%f"

rem ==== JLink 运行日志、EventRecorder 桩、uVision 窗口布局（均为自动再生文件）====
del /s /q JLinkLog.txt 2>nul
del /s /q EventRecorderStub.scvd 2>nul
del /s /q *.uvguix.* 2>nul

rem ==== Python 字节码缓存（webconsole / .pi_work 等运行残留，可整体删除）====
for /d /r %%d in (__pycache__) do if exist "%%d\" rd /s /q "%%d"

exit
