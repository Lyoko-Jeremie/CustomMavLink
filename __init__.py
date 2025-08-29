#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CustomMavLink - 基于Python的无人机管理系统

这个包提供了无人机对象管理和MavLink协议通信功能。

主要模块:
- airplane_owl02: 无人机对象类
- airplane_manager_owl02: 无人机管理类  
- main: 基础协议处理
- owl02: OWL02协议实现
"""

from .version import __version__, __author__, __email__, __description__

# 导入主要类和函数
try:
    from .airplane_owl02 import AirplaneOwl02
    from .airplane_manager_owl02 import AirplaneManagerOwl02, create_manager, create_manager_with_serial
    from .main import wrap_packet, unwrap_packet, calculate_checksum
except ImportError:
    # 如果导入失败，可能是因为依赖未安装
    pass

__all__ = [
    '__version__',
    '__author__', 
    '__email__',
    '__description__',
    'AirplaneOwl02',
    'AirplaneManagerOwl02',
    'create_manager',
    'create_manager_with_serial',
    'wrap_packet',
    'unwrap_packet', 
    'calculate_checksum',
]
