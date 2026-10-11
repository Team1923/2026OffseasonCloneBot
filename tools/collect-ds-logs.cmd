@echo off
rem Double-click launcher for collect-ds-logs.sh (runs it in Git Bash). Arguments pass through.
setlocal
set "BASH=%ProgramFiles%\Git\bin\bash.exe"
if not exist "%BASH%" set "BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not exist "%BASH%" set "BASH=%LocalAppData%\Programs\Git\bin\bash.exe"
if not exist "%BASH%" (
  echo Git Bash not found. Install Git for Windows: https://git-scm.com/download/win
  pause
  exit /b 1
)
"%BASH%" "%~dp0collect-ds-logs.sh" %*
echo.
pause
