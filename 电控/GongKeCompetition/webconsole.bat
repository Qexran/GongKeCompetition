@echo off
rem GKS Web 控制台启动器: 起服务(端口 8410, 绑 0.0.0.0)并自动打开浏览器
rem 启动后终端会打印「局域网」那条地址 —— 发给队友, 他们照抄进浏览器即可同时用。
cd /d %~dp0
python tools\webconsole\server.py
pause
