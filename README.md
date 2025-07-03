1. 下拉代码

git clone https://github.com/Hardware-01-ST-F411CE-Weact-25M/07-gg-all-inone.git

2. 忽略ConEmu.xml的修改

cd tools/env_win

git update-index --assume-unchanged tools/ConEmu/ConEmu.xml

git update-index --assume-unchanged .vscode/settings.json

git update-index --assume-unchanged .vscode/launch.json

3. 下载大文件
   git lfs install
   git lfs pull
