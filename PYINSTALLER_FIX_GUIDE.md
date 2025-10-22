# PyInstaller 打包问题解决方案

## 问题描述
打包后的exe运行时出现错误：
```
ModuleNotFoundError: No module named 'owl2'
```

## 解决方案

我已经修复了所有打包配置文件，问题的根源是PyInstaller无法自动识别本地的`owl2`模块。

### 修改内容

#### 1. **build_pair_tool.spec**（推荐使用）
已更新为包含以下关键配置：
- 添加项目根目录到`pathex`
- 使用`datas`参数将整个`owl2`目录打包进去
- 添加所有`owl2`子模块到`hiddenimports`
- 添加`serial`相关模块到隐藏导入

#### 2. **build_pair_tool_simple.bat**
已更新命令行参数：
- 使用`--add-data="owl2;owl2"`将模块目录打包
- 添加所有必要的`--hidden-import`参数

#### 3. **build_pair_tool_debug.bat**（新增）
创建了调试版本打包脚本：
- 显示控制台窗口，方便查看错误信息
- 清理旧的打包文件
- 包含完整的模块导入配置

## 使用方法

### 方法1：使用spec文件打包（推荐）

```bash
# 双击运行或在命令行执行
build_pair_tool.bat
```

这会：
1. 自动安装依赖（pyinstaller、pyserial、pymavlink）
2. 使用优化的spec配置文件打包
3. 生成：`dist\无人机配对工具.exe`

### 方法2：使用简易脚本打包

```bash
# 双击运行或在命令行执行
build_pair_tool_simple.bat
```

### 方法3：调试版本打包（遇到问题时使用）

```bash
# 双击运行或在命令行执行
build_pair_tool_debug.bat
```

调试版本会：
- 显示控制台窗口
- 输出详细的错误信息
- 帮助定位问题

## 打包前检查清单

1. ✅ 确保在项目根目录下运行打包脚本
2. ✅ 确保`owl2`文件夹存在且包含所有必要文件
3. ✅ 确保虚拟环境已激活（如果使用）
4. ✅ 确保已安装所有依赖：
   ```bash
   pip install pyinstaller pyserial pymavlink
   ```

## 关键配置说明

### spec文件中的关键部分

```python
# 1. 添加项目路径
pathex=[project_root]

# 2. 打包owl2目录
datas=[
    ('owl2', 'owl2'),  # 整个目录
    ('owl2/*.py', 'owl2'),  # 所有Python文件
    ('owl2/commonACFly/*.py', 'owl2/commonACFly'),  # 子目录
]

# 3. 隐藏导入
hiddenimports=[
    'owl2',
    'owl2.pair_manager',
    'owl2.custom_protocol_packet',
    'owl2.commonACFly',
    'owl2.commonACFly.commonACFly_py3',
    # ... 其他子模块
]
```

### 命令行参数说明

```bash
--add-data="owl2;owl2"  # Windows语法：源目录;目标目录
--hidden-import=owl2    # 明确导入模块
--paths=.               # 添加当前目录到搜索路径
```

## 验证打包是否成功

### 1. 检查文件大小
成功打包的exe文件应该在10-30MB左右（包含了所有依赖）

### 2. 运行测试
双击运行 `dist\无人机配对工具.exe`，应该能看到GUI界面

### 3. 如果仍有问题
运行调试版本：
```bash
build_pair_tool_debug.bat
```
然后运行 `dist\无人机配对工具_调试版.exe`，查看控制台输出的错误信息

## 常见问题

### Q1: 打包后exe文件很小（几KB）
**A:** 打包失败，检查是否有错误信息。使用调试模式重新打包。

### Q2: 提示"找不到xxx模块"
**A:** 在spec文件的`hiddenimports`中添加该模块，或使用`--hidden-import=模块名`

### Q3: 打包速度很慢
**A:** 正常现象，首次打包需要5-10分钟。后续使用`--clean`会更快。

### Q4: UPX压缩失败
**A:** 在spec文件中设置`upx=False`禁用UPX压缩（会增大文件体积）

## 清理打包文件

如果需要完全重新打包，删除以下目录/文件：
```bash
rmdir /s /q build
rmdir /s /q dist
del *.spec
del /s /q __pycache__
```

或使用调试脚本，它会自动清理。

## 分发给用户

打包成功后，只需要分发：
```
dist\无人机配对工具.exe
```

用户无需：
- 安装Python
- 安装任何依赖库
- 配置环境变量

直接双击exe即可运行！

## 技术支持

如果按照以上步骤仍然无法解决问题，请检查：
1. Python版本（建议3.8-3.11）
2. PyInstaller版本（建议5.0+）
3. 是否在虚拟环境中打包
4. 杀毒软件是否拦截

---
更新日期：2025-10-22

