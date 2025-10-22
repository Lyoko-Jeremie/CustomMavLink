@echo off
chcp 65001
echo ================================
echo 无人机配对工具打包脚本
echo ================================
echo.

echo 检查依赖...
pip install pyinstaller pyserial pymavlink

echo.
echo 开始打包...
pyinstaller --clean build_pair_tool.spec

echo.
echo ================================
echo 打包完成！
echo 可执行文件位置: dist\无人机配对工具.exe
echo ================================
pause

