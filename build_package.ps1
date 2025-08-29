# CustomMavLink 打包脚本
# PowerShell 版本

Write-Host "======================================" -ForegroundColor Green
Write-Host "构建 CustomMavLink 包" -ForegroundColor Green  
Write-Host "======================================" -ForegroundColor Green

# 检查 Python 和必要工具
Write-Host "检查 Python 环境..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    Write-Host "Python 版本: $pythonVersion" -ForegroundColor Cyan
} catch {
    Write-Host "错误: 未找到 Python，请先安装 Python" -ForegroundColor Red
    exit 1
}

# 检查打包工具
Write-Host "检查打包工具..." -ForegroundColor Yellow
$checkTools = python -c "import setuptools, wheel, build" 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "安装打包工具..." -ForegroundColor Yellow
    pip install setuptools wheel build
}

# 清理旧文件
Write-Host "清理旧的构建文件..." -ForegroundColor Yellow
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }
Get-ChildItem -Directory -Name "*.egg-info" | ForEach-Object { Remove-Item -Recurse -Force $_ }

# 方法1: 使用传统的 setup.py
Write-Host "使用 setup.py 构建包..." -ForegroundColor Yellow
python setup.py sdist bdist_wheel

# 方法2: 使用现代的 build 工具（注释掉，可选择使用）
# Write-Host "使用 build 工具构建包..." -ForegroundColor Yellow
# python -m build

# 检查构建结果
if (Test-Path "dist") {
    Write-Host "======================================" -ForegroundColor Green
    Write-Host "构建完成！生成的文件：" -ForegroundColor Green
    Get-ChildItem "dist" | Format-Table Name, Length, LastWriteTime
    
    Write-Host "======================================" -ForegroundColor Green
    Write-Host "后续操作：" -ForegroundColor Cyan
    Write-Host "1. 安装本地包：" -ForegroundColor White
    $wheelFile = Get-ChildItem "dist/*.whl" | Select-Object -First 1
    if ($wheelFile) {
        Write-Host "   pip install $($wheelFile.FullName)" -ForegroundColor Gray
    }
    
    Write-Host ""
    Write-Host "2. 上传到 PyPI（可选）：" -ForegroundColor White
    Write-Host "   pip install twine" -ForegroundColor Gray
    Write-Host "   twine check dist/*" -ForegroundColor Gray
    Write-Host "   twine upload dist/*" -ForegroundColor Gray
    
    Write-Host ""
    Write-Host "3. 安装开发版本：" -ForegroundColor White
    Write-Host "   pip install -e ." -ForegroundColor Gray
    Write-Host "======================================" -ForegroundColor Green
} else {
    Write-Host "构建失败！请检查错误信息。" -ForegroundColor Red
    exit 1
}

Write-Host "按任意键继续..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
