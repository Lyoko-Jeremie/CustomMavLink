"""
无人机接口定义 - 抽象基类
定义了无人机对象的标准接口，所有无人机实现都应继承此接口
"""
from abc import ABC, abstractmethod
from typing import Dict, Optional, Any
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum


# 飞行模式枚举
class FlyModeEnum(Enum):
    """飞行主模式枚举"""
    FLY_MODE_HOLD = 2
    FLY_MODE_POSITION = 3
    FLY_MODE_AUTO = 4
    FLY_MODE_OFF_BOARD = 4
    INVALID = 16


class FlyModeAutoEnum(Enum):
    """自动飞行模式枚举"""
    FLY_MODE_AUTO_TAKEOFF = 2
    FLY_MODE_AUTO_FOLLOW = 3
    FLY_MODE_AUTO_MISSION = 4
    FLY_MODE_AUTO_RTL = 5
    FLY_MODE_AUTO_LAND = 6
    INVALID = 16


class FlyModeStableEnum(Enum):
    """稳定飞行模式枚举"""
    FLY_MODE_STABLE_NORMAL = 0
    FLY_MODE_STABLE_OBSTACLE_AVOIDANCE = 2
    INVALID = 16


@dataclass
class GpsPosition:
    """GPS位置信息"""
    lat: float = 0.0  # 纬度
    lon: float = 0.0  # 经度
    alt: float = 0.0  # 海拔高度（米）
    relative_alt: float = 0.0  # 相对高度（米）
    hdg: int = 0  # 航向角


@dataclass
class MavLinkPacketRecord:
    """MavLink包记录"""
    timestamp: datetime
    msg_id: int
    message: Any
    raw_packet: bytes


@dataclass
class AirplaneState:
    """无人机状态类"""
    is_armed: bool = False  # 是否解锁
    fly_mode: FlyModeEnum = FlyModeEnum.INVALID  # 飞行主模式
    fly_mode_auto: FlyModeAutoEnum = FlyModeAutoEnum.INVALID  # 自动飞行子模式
    fly_mode_stable: FlyModeStableEnum = FlyModeStableEnum.INVALID  # 稳定飞行子模式
    is_landed: int = 0  # 着陆状态
    flight_sw_version: Optional[int] = None  # 飞控软件版本号
    flight_sw_version_string: Optional[str] = None  # 飞控软件版本字符串
    board_version: Optional[int] = None  # 板卡版本号
    sn: Optional[str] = None  # 序列号
    gps_position: GpsPosition = field(default_factory=GpsPosition)  # GPS位置信息

    def __post_init__(self):
        if self.gps_position is None:
            self.gps_position = GpsPosition()


class IAirplane(ABC):
    """
    无人机接口抽象基类
    定义了所有无人机实现必须提供的标准接口

    实例变量：
        target_channel_id (int): 目标通道ID
        state (AirplaneState): 无人机状态对象
        is_init (bool): 是否已初始化
    """

    @abstractmethod
    def init(self) -> None:
        """
        初始化无人机
        在使用无人机前必须调用此方法
        """
        pass

    @abstractmethod
    def send_msg(self, msg: Any) -> bool:
        """
        发送消息给无人机
        :param msg: 要发送的消息对象
        :return: 发送是否成功
        """
        pass

    @abstractmethod
    def send_heartbeat(self) -> None:
        """
        发送心跳包
        用于保持与无人机的连接
        """
        pass

    @abstractmethod
    def parse_state_from_mavlink(self, message: Any, raw_packet: bytes = b'') -> None:
        """
        从MavLink消息解析状态
        :param message: MavLink消息对象
        :param raw_packet: 原始数据包（可选）
        """
        pass

    # ==================== 状态查询接口 ====================

    @abstractmethod
    def get_state(self) -> AirplaneState:
        """
        获取无人机当前状态
        :return: 无人机状态对象
        """
        pass

    @abstractmethod
    def get_gps_pos(self) -> Optional[Dict[str, float]]:
        """
        获取GPS位置信息
        :return: 包含位置信息的字典，如果未获取到则返回None
                 字典包含: lat, lon, alt, relative_alt, vx, vy, vz, hdg
        """
        pass

    @abstractmethod
    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        获取姿态信息
        :return: 包含姿态信息的字典，如果未获取到则返回None
                 字典包含: roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        """
        pass

    @abstractmethod
    def get_cached_packet(self, msg_id: int) -> Optional[Any]:
        """
        获取缓存的数据包
        :param msg_id: 消息ID
        :return: 缓存的数据包记录，如果不存在则返回None
        """
        pass

    @abstractmethod
    def trigger_get_autopilot_version(self) -> bool:
        """
        触发获取自动驾驶仪版本信息
        :return: 请求是否发送成功
        """
        pass

    # ==================== 基础控制接口 ====================

    @abstractmethod
    def arm(self) -> None:
        """
        解锁无人机
        解锁后电机才能启动
        """
        pass

    @abstractmethod
    def disarm(self) -> None:
        """
        锁定无人机
        锁定后电机停止
        """
        pass

    @abstractmethod
    def takeoff(self, altitude: float) -> None:
        """
        起飞到指定高度
        :param altitude: 目标高度（米）
        """
        pass

    @abstractmethod
    def land(self) -> None:
        """
        降落
        无人机将在当前位置降落
        """
        pass

    @abstractmethod
    def return_to_launch(self) -> None:
        """
        返航
        无人机将返回起飞点
        """
        pass

    # ==================== 位置控制接口 ====================

    @abstractmethod
    def up(self, distance: int) -> None:
        """
        上升指定距离
        :param distance: 上升距离（厘米）
        """
        pass

    @abstractmethod
    def down(self, distance: int) -> None:
        """
        下降指定距离
        :param distance: 下降距离（厘米）
        """
        pass

    @abstractmethod
    def forward(self, distance: int) -> None:
        """
        前进指定距离
        :param distance: 前进距离（厘米）
        """
        pass

    @abstractmethod
    def back(self, distance: int) -> None:
        """
        后退指定距离
        :param distance: 后退距离（厘米）
        """
        pass

    @abstractmethod
    def left(self, distance: int) -> None:
        """
        左移指定距离
        :param distance: 左移距离（厘米）
        """
        pass

    @abstractmethod
    def right(self, distance: int) -> None:
        """
        右移指定距离
        :param distance: 右移距离（厘米）
        """
        pass

