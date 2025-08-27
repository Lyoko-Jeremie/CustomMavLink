"""
无人机对象类，参考TypeScript的AirplaneOwl02实现
"""
import time
from datetime import datetime
from typing import Dict, Optional, Any, Callable
from enum import Enum
from dataclasses import dataclass, field
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import threading
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# 飞行模式枚举
class FlyModeEnum(Enum):
    FLY_MODE_HOLD = 2
    FLY_MODE_POSITION = 3
    FLY_MODE_AUTO = 4
    FLY_MODE_OFF_BOARD = 4
    INVALID = 16


class FlyModeAutoEnum(Enum):
    FLY_MODE_AUTO_TAKEOFF = 2
    FLY_MODE_AUTO_FOLLOW = 3
    FLY_MODE_AUTO_MISSION = 4
    FLY_MODE_AUTO_RTL = 5
    FLY_MODE_AUTO_LAND = 6
    INVALID = 16


class FlyModeStableEnum(Enum):
    FLY_MODE_STABLE_NORMAL = 0
    FLY_MODE_STABLE_OBSTACLE_AVOIDANCE = 2
    INVALID = 16


@dataclass
class GpsPosition:
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    relative_alt: float = 0.0
    hdg: int = 0


@dataclass
class AirplaneState:
    """无人机状态类"""
    is_armed: bool = False
    fly_mode: FlyModeEnum = FlyModeEnum.INVALID
    fly_mode_auto: FlyModeAutoEnum = FlyModeAutoEnum.INVALID
    fly_mode_stable: FlyModeStableEnum = FlyModeStableEnum.INVALID
    is_landed: int = mavlink2.MAV_LANDED_STATE_UNDEFINED
    flight_sw_version: Optional[int] = None
    flight_sw_version_string: Optional[str] = None
    board_version: Optional[int] = None
    sn: Optional[str] = None
    gps_position: GpsPosition = field(default_factory=GpsPosition)


@dataclass
class MavLinkPacketRecord:
    """MavLink包记录"""
    timestamp: datetime
    msg_id: int
    message: Any
    raw_packet: bytes


class AirplaneOwl02:
    """无人机对象类"""

    def __init__(self, target_channel_id: int, manager: 'AirplaneManagerOwl02'):
        self.target_channel_id = target_channel_id
        self.manager = manager
        self.state = AirplaneState()

        # 缓存最后接收到的每种MavLink包
        self.cached_packet_record: Dict[int, MavLinkPacketRecord] = {}

        # 消息解析表
        self.parse_table: Dict[int, Callable[[Any], None]] = {
            mavlink2.MAVLINK_MSG_ID_HEARTBEAT: self._parse_heartbeat,
            mavlink2.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: self._parse_land_state,
            mavlink2.MAVLINK_MSG_ID_AUTOPILOT_VERSION: self._parse_autopilot_version,
            mavlink2.MAVLINK_MSG_ID_STATUSTEXT: self._parse_status_text,
            mavlink2.MAVLINK_MSG_ID_COMMAND_ACK: self._parse_ack,
            mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: self._parse_gps_pos,
            mavlink2.MAVLINK_MSG_ID_BATTERY_STATUS: self._parse_battery_status,
        }

        # 需要缓存的包ID集合
        self.cached_packet_ids = {
            mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavlink2.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavlink2.MAVLINK_MSG_ID_GPS2_RAW,
            mavlink2.MAVLINK_MSG_ID_VFR_HUD,
            mavlink2.MAVLINK_MSG_ID_ATTITUDE,
            mavlink2.MAVLINK_MSG_ID_RC_CHANNELS,
            mavlink2.MAVLINK_MSG_ID_RC_CHANNELS_SCALED,
            mavlink2.MAVLINK_MSG_ID_MISSION_CURRENT,
            mavlink2.MAVLINK_MSG_ID_BATTERY_STATUS,
        }

        self.is_init = False
        self._lock = threading.Lock()

    async def init(self):
        """初始化无人机"""
        if self.is_init:
            return
        self.is_init = True
        logger.info(f"Initializing airplane with ID: {self.target_channel_id}")
        await self.send_heartbeat()

    async def send_msg(self, msg):
        """发送消息给无人机"""
        return await self.manager.send_msg(msg, self.target_channel_id)

    async def send_heartbeat(self):
        """发送心跳包"""
        heartbeat = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_GCS,
            autopilot=mavlink2.MAV_AUTOPILOT_GENERIC,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink2.MAV_STATE_ACTIVE,
            mavlink_version=2,
        )
        await self.send_msg(heartbeat)
        logger.debug(f"Sent heartbeat to device {self.target_channel_id}")

    async def trigger_get_autopilot_version(self):
        """触发获取自动驾驶仪版本信息"""
        # 发送请求自动驾驶仪能力的命令
        request_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            confirmation=0,
            param1=1,  # 请求版本信息
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        return await self.send_msg(request_cmd)

    def _cache_packet_record(self, msg_id: int, message: Any, raw_packet: bytes = b''):
        """缓存数据包记录"""
        with self._lock:
            record = MavLinkPacketRecord(
                timestamp=datetime.now(),
                msg_id=msg_id,
                message=message,
                raw_packet=raw_packet
            )
            self.cached_packet_record[msg_id] = record

    def _parse_heartbeat(self, message: mavlink2.MAVLink_heartbeat_message):
        """解析心跳包"""
        self.state.is_armed = (message.base_mode & 0x80) == 0x80

        # 解析飞行模式
        main_mode = (message.custom_mode >> (8 * 3)) & 0xFF
        sub_mode = (message.custom_mode >> (8 * 4)) & 0xFF

        mode_map = {
            2: FlyModeEnum.FLY_MODE_HOLD,
            3: FlyModeEnum.FLY_MODE_POSITION,
            4: FlyModeEnum.FLY_MODE_AUTO,
        }
        self.state.fly_mode = mode_map.get(main_mode, FlyModeEnum.INVALID)

        # 根据主模式解析子模式
        if self.state.fly_mode == FlyModeEnum.FLY_MODE_AUTO:
            auto_mode_map = {
                2: FlyModeAutoEnum.FLY_MODE_AUTO_TAKEOFF,
                3: FlyModeAutoEnum.FLY_MODE_AUTO_FOLLOW,
                4: FlyModeAutoEnum.FLY_MODE_AUTO_MISSION,
                5: FlyModeAutoEnum.FLY_MODE_AUTO_RTL,
                6: FlyModeAutoEnum.FLY_MODE_AUTO_LAND,
            }
            self.state.fly_mode_auto = auto_mode_map.get(sub_mode, FlyModeAutoEnum.INVALID)
            self.state.fly_mode_stable = FlyModeStableEnum.INVALID
        elif self.state.fly_mode == FlyModeEnum.FLY_MODE_POSITION:
            stable_mode_map = {
                0: FlyModeStableEnum.FLY_MODE_STABLE_NORMAL,
                2: FlyModeStableEnum.FLY_MODE_STABLE_OBSTACLE_AVOIDANCE,
            }
            self.state.fly_mode_stable = stable_mode_map.get(sub_mode, FlyModeStableEnum.INVALID)
            self.state.fly_mode_auto = FlyModeAutoEnum.INVALID
        else:
            self.state.fly_mode_auto = FlyModeAutoEnum.INVALID
            self.state.fly_mode_stable = FlyModeStableEnum.INVALID

    def _parse_land_state(self, message: mavlink2.MAVLink_extended_sys_state_message):
        """解析着陆状态"""
        self.state.is_landed = message.landed_state

    def _parse_status_text(self, message: mavlink2.MAVLink_statustext_message):
        """解析状态文本"""
        text = message.text.decode('utf-8').strip('\x00')
        logger.info(f"Status text from device {self.target_channel_id}: {text}")

    def _parse_autopilot_version(self, message: mavlink2.MAVLink_autopilot_version_message):
        """解析自动驾驶仪版本信息"""
        self.state.flight_sw_version = message.flight_sw_version

        # 解析版本号字符串
        version_bytes = [
            (message.flight_sw_version >> (8 * 2)) & 0xFF,
            (message.flight_sw_version >> (8 * 1)) & 0xFF,
            (message.flight_sw_version >> (8 * 0)) & 0xFF,
        ]
        self.state.flight_sw_version_string = '.'.join(map(str, version_bytes))
        self.state.board_version = message.board_version

        # 解析序列号
        if hasattr(message, 'uid2') and len(message.uid2) >= 3:
            uid_parts = []
            for i in range(3):
                uid_part = (message.uid2[i] & 0xFFFFFFFF)
                uid_parts.append(f"{uid_part:08x}")
            self.state.sn = ''.join(uid_parts)

    def _parse_ack(self, message: mavlink2.MAVLink_command_ack_message):
        """解析命令确认"""
        logger.info(f"Command ACK from device {self.target_channel_id}: "
                    f"command={message.command}, result={message.result}")

    def _parse_gps_pos(self, message: mavlink2.MAVLink_global_position_int_message):
        """解析GPS位置"""
        self.state.gps_position.lat = message.lat / 1e7
        self.state.gps_position.lon = message.lon / 1e7
        self.state.gps_position.alt = message.alt / 1e3
        self.state.gps_position.relative_alt = message.relative_alt / 1e3
        self.state.gps_position.hdg = message.hdg

    def _parse_battery_status(self, message: mavlink2.MAVLink_battery_status_message):
        """解析电池状态"""
        logger.debug(f"Battery status from device {self.target_channel_id}: "
                     f"voltage={message.voltages}, current={message.current_battery}, "
                     f"remaining={message.battery_remaining}")

    def parse_state_from_mavlink(self, message: Any, raw_packet: bytes = b''):
        """从MavLink消息解析状态"""
        msg_id = message.get_msgId()

        # 缓存数据包
        self._cache_packet_record(msg_id, message, raw_packet)

        # 查找并调用对应的解析函数
        parse_func = self.parse_table.get(msg_id)
        if parse_func:
            try:
                parse_func(message)
            except Exception as e:
                logger.error(f"Error parsing message {msg_id}: {e}")
        else:
            if msg_id not in self.cached_packet_ids:
                logger.warning(f"Unknown message ID {msg_id} from device {self.target_channel_id}")

    def get_gps_pos(self) -> Optional[Dict[str, float]]:
        """获取GPS位置信息"""
        record = self.cached_packet_record.get(mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        if not record:
            return None

        msg = record.message
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.alt / 1e3,
            'relative_alt': msg.relative_alt / 1e3,
            'vx': msg.vx,
            'vy': msg.vy,
            'vz': msg.vz,
            'hdg': msg.hdg,
        }

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """获取姿态信息"""
        record = self.cached_packet_record.get(mavlink2.MAVLINK_MSG_ID_ATTITUDE)
        if not record:
            return None

        msg = record.message
        return {
            'roll': msg.roll,
            'pitch': msg.pitch,
            'yaw': msg.yaw,
            'rollspeed': msg.rollspeed,
            'pitchspeed': msg.pitchspeed,
            'yawspeed': msg.yawspeed,
        }

    def get_state(self) -> AirplaneState:
        """获取无人机状态"""
        return self.state

    def get_cached_packet(self, msg_id: int) -> Optional[MavLinkPacketRecord]:
        """获取缓存的数据包"""
        return self.cached_packet_record.get(msg_id)

    # 控制接口
    async def arm(self):
        """解锁无人机"""
        arm_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1,  # 1 = arm, 0 = disarm
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        await self.send_msg(arm_cmd)
        logger.info(f"Sent arm command to device {self.target_channel_id}")

    async def disarm(self):
        """锁定无人机"""
        disarm_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0,  # 1 = arm, 0 = disarm
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        await self.send_msg(disarm_cmd)
        logger.info(f"Sent disarm command to device {self.target_channel_id}")

    async def takeoff(self, altitude: float):
        """起飞到指定高度"""
        takeoff_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0,  # pitch
            param2=0,
            param3=0,
            param4=0,  # yaw
            param5=0,
            param6=0,
            param7=altitude  # altitude
        )
        await self.send_msg(takeoff_cmd)
        logger.info(f"Sent takeoff command to device {self.target_channel_id} at altitude {altitude}")

    async def land(self):
        """降落"""
        land_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_NAV_LAND,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        await self.send_msg(land_cmd)
        logger.info(f"Sent land command to device {self.target_channel_id}")

    async def return_to_launch(self):
        """返航"""
        rtl_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        await self.send_msg(rtl_cmd)
        logger.info(f"Sent RTL command to device {self.target_channel_id}")
