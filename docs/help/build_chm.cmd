@echo off
rem hhc.exe returns 1 on success
%1 %2
if errorlevel 1 exit /b 0
exit /b 1
