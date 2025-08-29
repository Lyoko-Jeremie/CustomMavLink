# CustomMavLink 打包和安装指南

这个文档说明如何打包和安装 CustomMavLink 项目。

## 文件说明

### 打包相关文件

- `setup.py` - 传统的 setuptools 配置文件
- `pyproject.toml` - 现代 Python 打包配置文件（PEP 518/517 标准）
- `version.py` - 版本信息
- `MANIFEST.in` - 指定打包时包含的额外文件
- `LICENSE` - MIT 许可证文件
- `__init__.py` - 包初始化文件

### 构建脚本

- `build_package.bat` - Windows 批处理构建脚本
- `build_package.ps1` - PowerShell 构建脚本
- `Makefile` - Linux/Mac 构建脚本

## 快速开始

### 1. 环境准备

确保已安装 Python 3.7+ 和必要的打包工具：

```powershell
# 安装打包工具
pip install setuptools wheel build twine

# 或者安装开发依赖
pip install -e ".[dev]"
```

### 2. 构建包

#### 方法 A: 使用 PowerShell 脚本（推荐）

```powershell
.\build_package.ps1
```

#### 方法 B: 使用批处理脚本

```cmd
build_package.bat
```

#### 方法 C: 手动构建

```powershell
# 清理旧文件
Remove-Item -Recurse -Force build, dist, *.egg-info -ErrorAction SilentlyContinue

# 传统方式构建
python setup.py sdist bdist_wheel

# 现代方式构建
python -m build
```

### 3. 安装包

#### 本地安装

```powershell
# 从源码安装
pip install .

# 开发模式安装（可编辑）
pip install -e .

# 从 wheel 文件安装
pip install dist\custom_mavlink-1.0.0-py3-none-any.whl
```

#### 从 PyPI 安装（如果已上传）

```powershell
pip install custom-mavlink
```

## 开发工作流

### 1. 设置开发环境

```powershell
# 克隆项目
git clone https://github.com/Lyoko-Jeremie/CustomMavLink.git
cd CustomMavLink

# 安装开发依赖
pip install -e ".[dev]"
```

### 2. 代码质量检查

```powershell
# 代码格式化
black *.py

# 代码检查
flake8 *.py
mypy *.py

# 运行测试
pytest
```

### 3. 构建和发布

```powershell
# 清理并构建
Remove-Item -Recurse -Force build, dist, *.egg-info -ErrorAction SilentlyContinue
python -m build

# 检查包
twine check dist/*

# 上传到测试 PyPI
twine upload --repository testpypi dist/*

# 上传到正式 PyPI
twine upload dist/*
```

## 包使用示例

安装后可以这样使用：

```python
# 导入包
import custom_mavlink
from custom_mavlink import AirplaneManagerOwl02, create_manager_with_serial

# 创建管理器
manager = create_manager_with_serial("COM3", 115200)

# 使用管理器...
```

## 命令行工具

安装后会提供命令行工具：

```powershell
custom-mavlink  # 运行主程序
```

## 故障排除

### 常见问题

1. **构建失败**: 确保安装了 setuptools 和 wheel
2. **导入错误**: 检查依赖是否正确安装
3. **权限错误**: 在 Windows 上可能需要管理员权限

### 检查安装

```python
import custom_mavlink
print(custom_mavlink.__version__)
print(custom_mavlink.__author__)
```

## 项目结构

```
CustomMavLink/
├── airplane_owl02.py          # 无人机对象类
├── airplane_manager_owl02.py  # 无人机管理类
├── main.py                    # 基础协议处理
├── owl02.py                   # OWL02协议
├── version.py                 # 版本信息
├── __init__.py                # 包初始化
├── setup.py                   # setuptools 配置
├── pyproject.toml             # 现代打包配置
├── MANIFEST.in                # 打包文件清单
├── LICENSE                    # 许可证
├── README.md                  # 项目说明
├── requirements.txt           # 依赖列表
├── build_package.ps1          # PowerShell 构建脚本
├── build_package.bat          # 批处理构建脚本
├── Makefile                   # Make 构建脚本
└── INSTALL.md                 # 本文件
```
