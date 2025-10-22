# -*- mode: python ; coding: utf-8 -*-
import os
import sys

block_cipher = None

# 获取项目根目录
project_root = os.path.abspath('.')

# 添加项目根目录到Python路径
sys.path.insert(0, project_root)

a = Analysis(
    ['pair_tools_gui.py'],
    pathex=[project_root],  # 添加项目根目录到搜索路径
    binaries=[],
    datas=[
        ('owl2', 'owl2'),  # 将owl2目录整体打包进去
        ('owl2/*.py', 'owl2'),  # 确保所有Python文件都被包含
        ('owl2/commonACFly/*.py', 'owl2/commonACFly'),  # 包含commonACFly子目录
    ],
    hiddenimports=[
        'owl2',
        'owl2.pair_manager',
        'owl2.custom_protocol_packet',
        'owl2.commonACFly',
        'owl2.commonACFly.commonACFly_py3',
        'owl2.airplane_interface',
        'owl2.airplane_manager_owl02',
        'owl2.airplane_owl02',
        'owl2.owl02',
        'serial',
        'serial.tools',
        'serial.tools.list_ports',
    ],
    hookspath=[project_root],  # 使用自定义钩子
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='无人机配对工具',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,  # 不显示控制台窗口
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,  # 可以添加图标文件路径
)
