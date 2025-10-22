@echo off
chcp 65001
echo ================================
echo 无人机配对工具简易打包脚本
echo ================================
echo.

echo 检查依赖...
pip install pyinstaller pyserial pymavlink

echo.
echo 开始打包...
pyinstaller --onefile --windowed ^
    --name="无人机配对工具" ^
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
    pair_tools_gui.py

echo.
echo ================================
echo 打包完成！
echo 可执行文件位置: dist\无人机配对工具.exe
echo ================================
pause
