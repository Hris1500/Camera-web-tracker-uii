@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    py -3 -m venv .venv
    if errorlevel 1 goto error
)
if not exist ".venv\detector-seguimiento-instalado" (
    echo Instalando las dependencias. Esto puede tardar unos minutos.
    ".venv\Scripts\python.exe" -m pip install ultralytics lap
    if errorlevel 1 goto error
    type nul > ".venv\detector-seguimiento-instalado"
)
".venv\Scripts\python.exe" detector.py %*
if errorlevel 1 goto error
exit /b 0

:error
echo.
echo Revisa el mensaje de arriba.
pause
exit /b 1


