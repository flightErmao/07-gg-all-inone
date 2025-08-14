@echo off
echo Building with baro-dps368 config...
copy configs\01-baro-dps368 .config
scons
pause