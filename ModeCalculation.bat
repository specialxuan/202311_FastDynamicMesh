@echo off
setlocal

rem Project root = directory of this script (with trailing backslash)
set "PROJECT_DIR=%~dp0"

rem MAPDL path: override with the MAPDL_EXE environment variable if needed
if not defined MAPDL_EXE set "MAPDL_EXE=C:\Program Files\ANSYS Inc\v221\ansys\bin\winx64\MAPDL.EXE"

if not exist "%MAPDL_EXE%" (
    echo [ERROR] MAPDL not found: "%MAPDL_EXE%"
    echo         Set the MAPDL_EXE environment variable to the correct path and retry.
    exit /b 1
)

"%MAPDL_EXE%" -lch -p ansys -smp -np 4 -dir "%PROJECT_DIR%mode" -j Mode -i "%PROJECT_DIR%mode\ModeCalculation.apdl" -o "%PROJECT_DIR%mode\Mode.out" -b -l en-us -s read

endlocal
