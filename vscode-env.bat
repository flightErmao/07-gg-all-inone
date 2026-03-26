@echo off

REM VSCode integrated terminal environment setup
REM Replaces ConEmu: calls env-init.bat directly without launching external window

set REPO_ROOT=%~dp0
set ENV_WIN_ROOT=%REPO_ROOT%tools\env_win\

REM ============= Git assume-unchanged setup =============
if exist "%ENV_WIN_ROOT%.git" (
    pushd "%ENV_WIN_ROOT%"
    if exist "tools\python-3.11.9-amd64\Lib\site-packages\pip-24.0.dist-info\RECORD" (
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Lib/site-packages/pip-24.0.dist-info/RECORD" 2>nul
    )
    if exist "tools\python-3.11.9-amd64\Scripts\pip3.11.exe" (
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Scripts/pip3.11.exe" 2>nul
    )
    if exist "tools\python-3.11.9-amd64\Scripts\pip3.exe" (
        git update-index --assume-unchanged "tools/python-3.11.9-amd64/Scripts/pip3.exe" 2>nul
    )
    if exist "tools\ConEmu\ConEmu.xml" (
        git update-index --assume-unchanged "tools/ConEmu/ConEmu.xml" 2>nul
    )
    popd
)

if exist "%REPO_ROOT%.vscode\settings.json" (
    git update-index --assume-unchanged ".vscode/settings.json" 2>nul
)
if exist "%REPO_ROOT%.vscode\launch.json" (
    git update-index --assume-unchanged ".vscode/launch.json" 2>nul
)

REM ============= Initialize RT-Thread Env (no ConEmu) =============
call "%ENV_WIN_ROOT%tools\bin\env-init.bat"
