"""
OWL02无人机控制包
提供基于BR&XGF控制协议2.0的无人机控制功能
"""

# 主要控制类
from .owl02 import Owl02

# 无人机管理类
from .airplane_manager_owl02 import (
    AirplaneManagerOwl02,
    get_airplane_manager_owl02,
    create_manager,
    create_manager_with_serial
)

# 无人机对象类
from .airplane_owl02 import AirplaneOwl02, CommandStatus

# 接口和数据类型
from .airplane_interface import (
    IAirplane,
    FlyModeEnum,
    FlyModeAutoEnum,
    FlyModeStableEnum,
    GpsPosition,
    MavLinkPacketRecord,
    AirplaneState
)

# 协议包处理工具
from .custom_protocol_packet import (
    PacketParser,
    wrap_packet,
    send_mavlink_packet_by_custom_protocol,
    send_raw_packet,
    receive_mavlink_packet
)

__all__ = [
    # 主要控制类
    'Owl02',

    # 无人机管理
    'AirplaneManagerOwl02',
    'get_airplane_manager_owl02',
    'create_manager',
    'create_manager_with_serial',

    # 无人机对象
    'AirplaneOwl02',
    'CommandStatus',

    # 接口和枚举
    'IAirplane',
    'FlyModeEnum',
    'FlyModeAutoEnum',
    'FlyModeStableEnum',

    # 数据类
    'GpsPosition',
    'MavLinkPacketRecord',
    'AirplaneState',

    # 协议包工具
    'PacketParser',
    'wrap_packet',
    'send_mavlink_packet_by_custom_protocol',
    'send_raw_packet',
    'receive_mavlink_packet',
]

__version__ = '2.0.0'

