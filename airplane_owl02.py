"""
无人机对象类，参考TypeScript的AirplaneOwl02实现
"""
import time
from datetime import datetime
from typing import Dict, Optional, Any, Callable
from enum import Enum
from dataclasses import dataclass, field
# from pymavlink.dialects.v20 import common as mavlink2
from commonACFly import commonACFly_py3 as mavlink2
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

    def init(self):
        """初始化无人机"""
        if self.is_init:
            return
        self.is_init = True
        logger.info(f"Initializing airplane with ID: {self.target_channel_id}")
        self.send_heartbeat()

    def send_msg(self, msg):
        """发送消息给无人机"""
        return self.manager.send_msg(msg, self.target_channel_id)

    def send_heartbeat(self):
        """发送心跳包"""
        heartbeat = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_GCS,
            autopilot=mavlink2.MAV_AUTOPILOT_GENERIC,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink2.MAV_STATE_ACTIVE,
            mavlink_version=2,
        )
        self.send_msg(heartbeat)
        logger.debug(f"Sent heartbeat to device {self.target_channel_id}")

    def trigger_get_autopilot_version(self):
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
        return self.send_msg(request_cmd)

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

    def disarm(self):
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
        self.send_msg(disarm_cmd)
        logger.info(f"Sent disarm command to device {self.target_channel_id}")

    def takeoff(self, altitude: float):
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
        self.send_msg(takeoff_cmd)
        logger.info(f"Sent takeoff command to device {self.target_channel_id} at altitude {altitude}")

    def land(self):
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
        self.send_msg(land_cmd)
        logger.info(f"Sent land command to device {self.target_channel_id}")

    def return_to_launch(self):
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
        self.send_msg(rtl_cmd)
        logger.info(f"Sent RTL command to device {self.target_channel_id}")

    def up(self, distance: int):
        """上升指定距离 单位cm"""
        # 使用位置偏移命令
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用z位置
            x=0, y=0, z=-distance/100.0,  # 负值表示上升，转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent up command to device {self.target_channel_id}, distance: {distance}cm")

    def down(self, distance: int):
        """下降指定距离 单位cm"""
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用z位置
            x=0, y=0, z=distance/100.0,  # 正值表示下降，转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent down command to device {self.target_channel_id}, distance: {distance}cm")

    def forward(self, distance: int):
        """前进指定距离 单位cm"""
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用x位置
            x=distance/100.0, y=0, z=0,  # 转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent forward command to device {self.target_channel_id}, distance: {distance}cm")

    def back(self, distance: int):
        """后退指定距离 单位cm"""
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用x位置
            x=-distance/100.0, y=0, z=0,  # 负值表示后退，转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent back command to device {self.target_channel_id}, distance: {distance}cm")

    def left(self, distance: int):
        """左移指定距离 单位cm"""
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用y位置
            x=0, y=-distance/100.0, z=0,  # 负值表示左移，转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent left command to device {self.target_channel_id}, distance: {distance}cm")

    def right(self, distance: int):
        """右移指定距离 单位cm"""
        offset_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask=0b110111111000,  # 只使用y位置
            x=0, y=distance/100.0, z=0,  # 正值表示右移，转换为米
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(offset_cmd)
        logger.info(f"Sent right command to device {self.target_channel_id}, distance: {distance}cm")

    def goto(self, x: int, y: int, h: int):
        """移动到指定坐标处 单位cm"""
        position_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask=0b110111111000,  # 使用位置控制
            x=x/100.0, y=y/100.0, z=-h/100.0,  # 转换为米，z为负值
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(position_cmd)
        logger.info(f"Sent goto command to device {self.target_channel_id}, x: {x}, y: {y}, h: {h}")

    def rotate(self, degree: int):
        """旋转指定角度"""
        self.cw(degree)

    def cw(self, degree: int):
        """顺时针旋转指定角度"""
        yaw_rad = degree * 3.14159 / 180.0  # 转换为弧度
        yaw_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask=0b100111111111,  # 只使用yaw
            x=0, y=0, z=0,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=yaw_rad, yaw_rate=0
        )
        self.send_msg(yaw_cmd)
        logger.info(f"Sent cw command to device {self.target_channel_id}, degree: {degree}")

    def ccw(self, degree: int):
        """逆时针旋转指定角度"""
        yaw_rad = -degree * 3.14159 / 180.0  # 转换为弧度，负值表示逆时针
        yaw_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask=0b100111111111,  # 只使用yaw
            x=0, y=0, z=0,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=yaw_rad, yaw_rate=0
        )
        self.send_msg(yaw_cmd)
        logger.info(f"Sent ccw command to device {self.target_channel_id}, degree: {degree}")

    def speed(self, speed: int):
        """设置飞行速度"""
        # 通过参数设置速度
        speed_cmd = mavlink2.MAVLink_param_set_message(
            target_system=1,
            target_component=1,
            param_id=b'MPC_XY_VEL_MAX'.ljust(16, b'\x00'),  # 水平最大速度参数
            param_value=float(speed/100.0),  # 转换为m/s
            param_type=mavlink2.MAV_PARAM_TYPE_REAL32
        )
        self.send_msg(speed_cmd)
        logger.info(f"Sent speed command to device {self.target_channel_id}, speed: {speed}")

    def high(self, high: int):
        """移动到指定高度处 单位cm"""
        position_cmd = mavlink2.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask=0b110111111011,  # 只使用z位置
            x=0, y=0, z=-high/100.0,  # 转换为米，z为负值
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0
        )
        self.send_msg(position_cmd)
        logger.info(f"Sent high command to device {self.target_channel_id}, height: {high}cm")

    def led(self, r: int, g: int, b: int):
        """设置无人机led色彩"""
        # 使用LED控制命令
        led_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_1,  # 自定义命令用于LED控制
            confirmation=0,
            param1=1,  # LED mode: solid color
            param2=r,  # Red
            param3=g,  # Green
            param4=b,  # Blue
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(led_cmd)
        logger.info(f"Sent LED command to device {self.target_channel_id}, RGB: ({r}, {g}, {b})")

    def bln(self, r: int, g: int, b: int):
        """设置无人机led呼吸灯色彩"""
        led_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_2,  # 自定义命令用于LED呼吸灯
            confirmation=0,
            param1=2,  # LED mode: breathing
            param2=r,  # Red
            param3=g,  # Green
            param4=b,  # Blue
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(led_cmd)
        logger.info(f"Sent LED breathing command to device {self.target_channel_id}, RGB: ({r}, {g}, {b})")

    def rainbow(self, r: int, g: int, b: int):
        """设置无人机led彩虹色彩"""
        led_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_3,  # 自定义命令用于LED彩虹
            confirmation=0,
            param1=3,  # LED mode: rainbow
            param2=r,  # Red
            param3=g,  # Green
            param4=b,  # Blue
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(led_cmd)
        logger.info(f"Sent LED rainbow command to device {self.target_channel_id}, RGB: ({r}, {g}, {b})")

    def airplane_mode(self, mode: int):
        """设置无人机飞行模式
        :param mode: 1常规2巡线3跟随4单机编队 通常情况下使用模式4
        """
        # 映射模式到MAVLink飞行模式
        mode_map = {
            1: (FlyModeEnum.FLY_MODE_POSITION, 0),  # 常规模式 -> 定点模式
            2: (FlyModeEnum.FLY_MODE_AUTO, FlyModeAutoEnum.FLY_MODE_AUTO_FOLLOW.value),  # 巡线 -> 自动跟随
            3: (FlyModeEnum.FLY_MODE_AUTO, FlyModeAutoEnum.FLY_MODE_AUTO_FOLLOW.value),  # 跟随 -> 自动跟随
            4: (FlyModeEnum.FLY_MODE_AUTO, FlyModeAutoEnum.FLY_MODE_AUTO_MISSION.value),  # 单机编队 -> 自动任务
        }
        
        if mode in mode_map:
            main_mode, sub_mode = mode_map[mode]
            custom_mode = (main_mode.value << (8 * 3)) | (sub_mode << (8 * 4))
            
            mode_cmd = mavlink2.MAVLink_set_mode_message(
                target_system=1,
                base_mode=mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=custom_mode
            )
            self.send_msg(mode_cmd)
            logger.info(f"Sent flight mode command to device {self.target_channel_id}, mode: {mode}")
        else:
            logger.warning(f"Unknown flight mode: {mode}")

    def stop(self):
        """停桨"""
        # 发送紧急停止命令
        stop_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0,  # 0 = disarm
            param2=21196,  # 紧急停止的魔术数字
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(stop_cmd)
        logger.info(f"Sent emergency stop command to device {self.target_channel_id}")

    def hover(self):
        """悬停"""
        # 发送悬停命令
        hover_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_OVERRIDE_GOTO,
            confirmation=0,
            param1=mavlink2.MAV_GOTO_DO_HOLD,  # 悬停
            param2=mavlink2.MAV_GOTO_HOLD_AT_CURRENT_POSITION,  # 在当前位置悬停
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(hover_cmd)
        logger.info(f"Sent hover command to device {self.target_channel_id}")

    def flip_forward(self):
        """前翻"""
        flip_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_4,  # 自定义命令用于翻滚
            confirmation=0,
            param1=1,  # 前翻
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(flip_cmd)
        logger.info(f"Sent flip forward command to device {self.target_channel_id}")

    def flip_back(self):
        """后翻"""
        flip_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_4,  # 自定义命令用于翻滚
            confirmation=0,
            param1=2,  # 后翻
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(flip_cmd)
        logger.info(f"Sent flip back command to device {self.target_channel_id}")

    def flip_left(self):
        """左翻"""
        flip_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_4,  # 自定义命令用于翻滚
            confirmation=0,
            param1=3,  # 左翻
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(flip_cmd)
        logger.info(f"Sent flip left command to device {self.target_channel_id}")

    def flip_right(self):
        """右翻"""
        flip_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_USER_4,  # 自定义命令用于翻滚
            confirmation=0,
            param1=4,  # 右翻
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.send_msg(flip_cmd)
        logger.info(f"Sent flip right command to device {self.target_channel_id}")
