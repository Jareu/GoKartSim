@echo off
setlocal

rem Optional first argument selects Python executable; default to python on PATH.
set "PYTHON_EXE=%~1"
if "%PYTHON_EXE%"=="" set "PYTHON_EXE=python"

echo Installing GoKartSim requirements with %PYTHON_EXE%...
echo.
echo NOTE: Install the official Project Chrono Python bindings from
echo       https://projectchrono.org/download/ before running the simulator.
echo.
"%PYTHON_EXE%" -m pip install --upgrade pip
"%PYTHON_EXE%" -m pip install -r "%~dp0requirements.txt"

echo Done.
