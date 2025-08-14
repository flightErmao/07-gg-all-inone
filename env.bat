@echo off

REM Auto call tools/env_win/env.bat, support selecting project in target by argument
setlocal
set PROJ=%1

if "%PROJ%"=="" (
    set BSP_PATH=%~dp0target
) else if "%PROJ%"=="01" (
    set BSP_PATH=%~dp0target\01-minifly-8M
) else (
    echo Unknown project number: %PROJ%
    set BSP_PATH=%~dp0target
    @REM exit /b 1
)

if not exist "%BSP_PATH%" (
    echo Path %BSP_PATH% does not exist!
    exit /b 1
)

REM Set git assume-unchanged for specific files to ignore their changes
set REPO_ROOT=%~dp0
set ENV_WIN_ROOT=%REPO_ROOT%tools\env_win\

echo Setting git assume-unchanged for Python tool files in submodule...

REM Check if submodule exists and is initialized
if exist "%ENV_WIN_ROOT%.git" (
    echo Submodule tools/env_win found, setting assume-unchanged in submodule...
    
    REM Change to submodule directory and set assume-unchanged
    pushd "%ENV_WIN_ROOT%"
    
    REM Check and set assume-unchanged for RECORD file
    if exist "tools\python-3.11.9-amd64\Lib\site-packages\pip-24.0.dist-info\RECORD" (
        echo Found: tools/python-3.11.9-amd64/Lib/site-packages/pip-24.0.dist-info/RECORD
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Lib/site-packages/pip-24.0.dist-info/RECORD" 2>nul
        if %errorlevel% equ 0 (
            echo   - Successfully set assume-unchanged for RECORD file
        ) else (
            echo   - Warning: Failed to set assume-unchanged for RECORD file
        )
    ) else (
        echo   - RECORD file not found, skipping
    )

    REM Check and set assume-unchanged for pip3.11.exe
    if exist "tools\python-3.11.9-amd64\Scripts\pip3.11.exe" (
        echo Found: tools/python-3.11.9-amd64/Scripts/pip3.11.exe
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Scripts/pip3.11.exe" 2>nul
        if %errorlevel% equ 0 (
            echo   - Successfully set assume-unchanged for pip3.11.exe
        ) else (
            echo   - Warning: Failed to set assume-unchanged for pip3.11.exe
        )
    ) else (
        echo   - pip3.11.exe not found, skipping
    )

    REM Check and set assume-unchanged for pip3.exe
    if exist "tools\python-3.11.9-amd64\Scripts\pip3.exe" (
        echo Found: tools/python-3.11.9-amd64/Scripts/pip3.exe
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Scripts/pip3.exe" 2>nul
        if %errorlevel% equ 0 (
            echo   - Successfully set assume-unchanged for pip3.exe
        ) else (
            echo   - Warning: Failed to set assume-unchanged for pip3.exe
        )
    ) else (
        echo   - pip3.exe not found, skipping
    )

    REM Check and set assume-unchanged for ConEmu.xml
    if exist "tools\ConEmu\ConEmu.xml" (
        echo Found: tools/ConEmu/ConEmu.xml
        git update-index --assume-unchanged "tools/ConEmu/ConEmu.xml" 2>nul
        if %errorlevel% equ 0 (
            echo   - Successfully set assume-unchanged for ConEmu.xml
        ) else (
            echo   - Warning: Failed to set assume-unchanged for ConEmu.xml
        )
    ) else (
        echo   - ConEmu.xml not found, skipping
    )
    
    popd
) else (
    echo Warning: Submodule tools/env_win not found or not initialized
    echo Please run: git submodule update --init --recursive
)

echo Git assume-unchanged setup completed.
echo.

REM Set git assume-unchanged for main repository files
echo Setting git assume-unchanged for main repository files...

REM Check and set assume-unchanged for .vscode/settings.json
if exist ".vscode\settings.json" (
    echo Found: .vscode/settings.json
    git update-index --assume-unchanged ".vscode/settings.json" 2>nul
    if %errorlevel% equ 0 (
        echo   - Successfully set assume-unchanged for settings.json
    ) else (
        echo   - Warning: Failed to set assume-unchanged for settings.json
    )
) else (
    echo   - settings.json not found, skipping
)

REM Check and set assume-unchanged for .vscode/launch.json
if exist ".vscode\launch.json" (
    echo Found: .vscode/launch.json
    git update-index --assume-unchanged ".vscode/launch.json" 2>nul
    if %errorlevel% equ 0 (
        echo   - Successfully set assume-unchanged for launch.json
    ) else (
        echo   - Warning: Failed to set assume-unchanged for launch.json
    )
) else (
    echo   - launch.json not found, skipping
)

echo Main repository assume-unchanged setup completed.
echo.

call tools\env_win\env.bat "%BSP_PATH%" 