@echo off
chcp 65001
echo ================================
echo 无人机配对工具调试版打包脚本
echo ================================
echo.

echo 清理旧的打包文件...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist
if exist "无人机配对工具.spec" del "无人机配对工具.spec"

echo.
echo 检查依赖...
pip install pyinstaller pyserial pymavlink

echo.
echo 开始打包（调试模式，显示控制台）...
pyinstaller --onefile ^
    --console ^
    --name="无人机配对工具_调试版" ^
    --add-data="owl2;owl2" ^
    --hidden-import=owl2 ^
    --hidden-import=owl2.pair_manager ^
    --hidden-import=owl2.custom_protocol_packet ^
    --hidden-import=owl2.commonACFly ^
    --hidden-import=owl2.commonACFly.commonACFly_py3 ^
    --hidden-import=owl2.airplane_interface ^
    --hidden-import=owl2.airplane_manager_owl02 ^
    --hidden-import=owl2.airplane_owl02 ^
    --hidden-import=owl2.owl02 ^
    --paths=. ^
    pair_tools_gui.py

echo.
echo ================================
echo 打包完成！
echo 可执行文件位置: dist\无人机配对工具_调试版.exe
echo 这是调试版本，会显示控制台窗口方便查看错误信息
echo ================================
pause

