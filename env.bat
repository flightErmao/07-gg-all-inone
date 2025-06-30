@echo off

REM Auto call tools/env_win/env.bat, support selecting project in bsp by argument
setlocal
set PROJ=%1

if "%PROJ%"=="" (
    set BSP_PATH=%~dp0bsp
) else if "%PROJ%"=="01" (
    set BSP_PATH=%~dp0bsp\01-minifly-8M
) else (
    echo Unknown project number: %PROJ%
    set BSP_PATH=%~dp0bsp
    @REM exit /b 1
)

if not exist "%BSP_PATH%" (
    echo Path %BSP_PATH% does not exist!
    exit /b 1
)
call tools\env_win\env.bat "%BSP_PATH%" 