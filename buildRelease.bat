@echo off
setlocal EnableExtensions
call "%~dp0build.bat" release %*
exit /b %ERRORLEVEL%
