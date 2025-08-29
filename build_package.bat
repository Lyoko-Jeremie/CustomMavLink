@echo off
echo ======================================
echo 构建 CustomMavLink 包
echo ======================================

REM 检查是否安装了必要的打包工具
echo 检查打包工具...
python -c "import setuptools, wheel" 2>nul
if errorlevel 1 (
    echo 安装打包工具...
    pip install setuptools wheel build
)

REM 清理之前的构建
echo 清理旧的构建文件...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist
if exist *.egg-info rmdir /s /q *.egg-info

REM 构建包
echo 开始构建包...
python setup.py sdist bdist_wheel

REM 检查构建结果
if exist dist (
    echo ======================================
    echo 构建完成！生成的文件：
    dir dist
    echo ======================================
    echo 安装本地包：
    echo pip install dist\custom_mavlink-1.0.0-py3-none-any.whl
    echo.
    echo 上传到PyPI（可选）：
    echo pip install twine
    echo twine upload dist/*
    echo ======================================
) else (
    echo 构建失败！请检查错误信息。
)

pause
