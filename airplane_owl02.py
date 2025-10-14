"""
无人机对象类，基于BR&XGF控制协议2.0实现
"""
from datetime import datetime
from typing import Dict, Optional, Any, Callable
from commonACFly import commonACFly_py3 as mavlink2
import threading
import logging
import time
from concurrent.futures import ThreadPoolExecutor
from airplane_interface import (
    IAirplane, FlyModeEnum, FlyModeAutoEnum, FlyModeStableEnum,
    AirplaneState, MavLinkPacketRecord
)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 命令应答结果常量（根据协议文档）
RECEIVE_COMMAND = 1  # 接收到命令
FINISH_COMMAND = 2   # 完成动作
COMMAND_ERROR = 3    # 拒绝执行指令


class CommandStatus:
    """命令状态追踪"""
    def __init__(self, command: int, sequence: int):
        self.command = command
        self.sequence = sequence  # 命令序列号，用于区分同一命令的不同调用
        self.receive_count = 0  # 接收应答计数
        self.finish_count = 0   # 完成应答计数
        self.is_received = False
        self.is_finished = False
        self.is_error = False
        self.last_update = time.time()
        self.create_time = time.time()


class AirplaneOwl02(IAirplane):
    """无人机对象类 - 基于BR&XGF控制协议2.0"""

    def __init__(self, target_channel_id: int, manager: 'AirplaneManagerOwl02'):
        self.target_channel_id = target_channel_id
        self.manager = manager
        self.state = AirplaneState()

        # 缓存最后接收到的每种MavLink包
        self.cached_packet_record: Dict[int, MavLinkPacketRecord] = {}

        # 命令状态追踪 - 使用 (command, sequence) 作为键
        self.command_status: Dict[tuple, CommandStatus] = {}
        self.command_lock = threading.Lock()
        self.command_sequence = 0  # 命令序列号生成器

        # 重发配置
        self.max_retries = 3  # 最大重发次数
        self.retry_timeout = 2.0  # 重发超时时间（秒）
        self.async_mode = True  # 异步模式：不阻塞等待应答

        # 线程池用于异步重发
        self.executor = ThreadPoolExecutor(max_workers=5, thread_name_prefix="cmd_retry")

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

    def __del__(self):
        """析构函数，清理线程池"""
        if hasattr(self, 'executor'):
            self.executor.shutdown(wait=False)

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
        request_cmd = mavlink2.MAVLink_command_long_message(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            confirmation=0,
            param1=1,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        return self.send_msg(request_cmd)

    def _get_next_sequence(self) -> int:
        """获取下一个命令序列号"""
        with self.command_lock:
            self.command_sequence += 1
            return self.command_sequence

    def _send_command_with_retry(self, command: int, param1=0, param2=0, param3=0,
                                  param4=0, param5=0, param6=0, param7=0,
                                  wait_for_finish=False, timeout=5.0, async_mode=None):
        """
        发送命令并自动重试（支持异步非阻塞模式）
        :param command: 命令ID
        :param param1-7: 命令参数
        :param wait_for_finish: 是否等待命令完成
        :param timeout: 等待超时时间
        :param async_mode: 是否异步模式（None时使用实例默认值）
        :return: 异步模式返回Future对象，同步模式返回是否成功
        """
        # 确定是否使用异步模式
        use_async = async_mode if async_mode is not None else self.async_mode

        # 生成命令序列号
        sequence = self._get_next_sequence()

        # 创建命令状态
        with self.command_lock:
            key = (command, sequence)
            self.command_status[key] = CommandStatus(command, sequence)

        # 定义实际执行重试的函数
        def _retry_task():
            retry_count = 0
            start_time = time.time()

            while retry_count < self.max_retries:
                # 发送命令
                cmd = mavlink2.MAVLink_command_long_message(
                    target_system=1,
                    target_component=1,
                    command=command,
                    confirmation=0,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    param4=param4,
                    param5=param5,
                    param6=param6,
                    param7=param7
                )
                self.send_msg(cmd)
                logger.debug(f"Sent command {command} seq={sequence} to device {self.target_channel_id} (attempt {retry_count + 1}/{self.max_retries})")

                # 等待应答
                wait_start = time.time()
                while time.time() - wait_start < self.retry_timeout:
                    with self.command_lock:
                        status = self.command_status.get(key)
                        if status:
                            if status.is_error:
                                logger.error(f"Command {command} seq={sequence} rejected by device {self.target_channel_id}")
                                return False

                            if status.is_received and not wait_for_finish:
                                logger.info(f"Command {command} seq={sequence} received by device {self.target_channel_id}")
                                return True

                            if status.is_finished:
                                logger.info(f"Command {command} seq={sequence} finished by device {self.target_channel_id}")
                                return True

                    # 检查总超时
                    if time.time() - start_time > timeout:
                        logger.warning(f"Command {command} seq={sequence} timeout after {timeout}s")
                        return False

                    time.sleep(0.05)  # 50ms检查间隔

                retry_count += 1
                if retry_count < self.max_retries:
                    logger.warning(f"Command {command} seq={sequence} no response, retrying... ({retry_count}/{self.max_retries})")

            logger.error(f"Command {command} seq={sequence} failed after {self.max_retries} retries")
            return False

        if use_async:
            # 异步模式：提交到线程池，立即返回Future对象
            future = self.executor.submit(_retry_task)
            logger.debug(f"Command {command} seq={sequence} submitted asynchronously")
            return future
        else:
            # 同步模式：阻塞等待完成
            return _retry_task()

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
        """
        解析命令确认
        根据协议文档：
        - RECEIVE_COMMAND = 1（接收到命令）- 会回复三次，200ms回复一次
        - FINISH_COMMAND = 2（完成动作）- 会发三次，500ms发一次
        - COMMAND_ERROR = 3（拒绝执行指令）
        """
        command = message.command
        result = message.result

        with self.command_lock:
            # 更新所有匹配该命令的状态（可能有多个序列号）
            updated = False
            for key, status in list(self.command_status.items()):
                if status.command == command:
                    status.last_update = time.time()

                    if result == RECEIVE_COMMAND:
                        status.receive_count += 1
                        if status.receive_count >= 1:
                            status.is_received = True
                        logger.debug(f"Command {command} seq={status.sequence} received ACK ({status.receive_count}/3)")
                        updated = True

                    elif result == FINISH_COMMAND:
                        status.finish_count += 1
                        if status.finish_count >= 1:
                            status.is_finished = True
                        logger.info(f"Command {command} seq={status.sequence} finished ACK ({status.finish_count}/3)")
                        updated = True

                    elif result == COMMAND_ERROR:
                        status.is_error = True
                        logger.error(f"Command {command} seq={status.sequence} error from device {self.target_channel_id}")
                        updated = True

            # 清理超过10秒的旧命令状态
            current_time = time.time()
            keys_to_remove = [k for k, v in self.command_status.items()
                            if current_time - v.create_time > 10.0]
            for k in keys_to_remove:
                del self.command_status[k]

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
                logger.debug(f"Unknown message ID {msg_id} from device {self.target_channel_id}")

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

    # ==================== 控制接口 - 使用协议文档中定义的命令 ====================

    def arm(self):
        """解锁无人机 - MAV_CMD_COMPONENT_ARM_DISARM (400)"""
        logger.info(f"Arming device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1  # 1 = arm
        )

    def disarm(self):
        """锁定无人机 - MAV_CMD_COMPONENT_ARM_DISARM (400)"""
        logger.info(f"Disarming device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0  # 0 = disarm
        )

    def takeoff(self, altitude: float):
        """
        起飞到指定高度 - MAV_CMD_EXT_DRONE_TAKEOFF (270)
        :param altitude: 起飞高度，单位米（会转换为cm）
        """
        height_cm = int(altitude * 100)  # 转换为cm
        # 限制范围: min值0，max值200cm
        height_cm = max(0, min(200, height_cm))
        logger.info(f"Taking off device {self.target_channel_id} to {height_cm}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_TAKEOFF,
            param1=height_cm,
            wait_for_finish=False,  # 改为非阻塞
            timeout=10.0
        )

    def land(self):
        """降落 - MAV_CMD_EXT_DRONE_LAND (271)"""
        logger.info(f"Landing device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_LAND,
            param1=1,    # 普通降落
            param2=100,  # 降落速度100cm/s
            wait_for_finish=False,  # 改为非阻塞
            timeout=15.0
        )

    def return_to_launch(self):
        """返航 - 使用标准命令"""
        logger.info(f"Returning to launch for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            wait_for_finish=False,  # 改为非阻塞
            timeout=20.0
        )

    def up(self, distance: int):
        """上升指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))  # 限制范围
        logger.info(f"Moving up device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=1,
            param2=distance,
            param3=100,  # 默认速度100cm/s
            wait_for_finish=False  # 改为非阻塞
        )

    def down(self, distance: int):
        """下降指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))
        logger.info(f"Moving down device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=2,
            param2=distance,
            param3=100,
            wait_for_finish=False
        )

    def forward(self, distance: int):
        """前进指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))
        logger.info(f"Moving forward device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=3,
            param2=distance,
            param3=100,
            wait_for_finish=False
        )

    def back(self, distance: int):
        """后退指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))
        logger.info(f"Moving back device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=4,
            param2=distance,
            param3=100,
            wait_for_finish=False
        )

    def left(self, distance: int):
        """左移指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))
        logger.info(f"Moving left device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=5,
            param2=distance,
            param3=100,
            wait_for_finish=False
        )

    def right(self, distance: int):
        """右移指定距离 - MAV_CMD_EXT_DRONE_MOVE (272)
        :param distance: 距离，单位cm
        """
        distance = max(0, min(1000, distance))
        logger.info(f"Moving right device {self.target_channel_id} by {distance}cm")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=6,
            param2=distance,
            param3=100,
            wait_for_finish=False
        )

    def goto(self, x: int, y: int, h: int):
        """
        移动到指定坐标处 - MAV_CMD_EXT_DRONE_WAYPOINT (274)
        :param x: x轴距离，单位cm（机头正前方为x轴正方向，min值-1000，max值1000）
        :param y: y轴距离，单位cm（机头左边为y轴正方向，min值-1000，max值1000）
        :param h: 飞行高度，单位cm（min值-200，max值200，最大高度2米）
        """
        x = max(-1000, min(1000, x))
        y = max(-1000, min(1000, y))
        h = max(-200, min(200, h))
        logger.info(f"Going to waypoint device {self.target_channel_id}: x={x}, y={y}, h={h}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_WAYPOINT,
            param1=x,
            param2=y,
            param3=h,
            wait_for_finish=False,  # 改为非阻塞
            timeout=10.0
        )

    def rotate(self, degree: int):
        """旋转指定角度 - 默认顺时针"""
        self.cw(degree)

    def cw(self, degree: int):
        """
        顺时针旋转指定角度 - MAV_CMD_EXT_DRONE_CIRCLE (273)
        :param degree: 角度（单位度，min值0，max值360）
        """
        degree = max(0, min(360, degree))
        logger.info(f"Rotating CW device {self.target_channel_id} by {degree} degrees")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_CIRCLE,
            param1=2,  # 顺时针
            param2=degree,
            wait_for_finish=False
        )

    def ccw(self, degree: int):
        """
        逆时针旋转指定角度 - MAV_CMD_EXT_DRONE_CIRCLE (273)
        :param degree: 角度（单位度，min值0，max值360）
        """
        degree = max(0, min(360, degree))
        logger.info(f"Rotating CCW device {self.target_channel_id} by {degree} degrees")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_CIRCLE,
            param1=1,  # 逆时针
            param2=degree,
            wait_for_finish=False
        )

    def speed(self, speed: int):
        """
        设置飞行速度 - MAV_CMD_EXT_DRONE_CHANGE_SPEED (275)
        :param speed: 速度，单位cm/s（min值0，max值200）
        """
        speed = max(0, min(200, speed))
        logger.info(f"Setting speed for device {self.target_channel_id} to {speed}cm/s")
        # param1: 飞行速度（单位cm/s）
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_CHANGE_SPEED,
            param1=speed
        )

    def high(self, high: int):
        """移动到指定高度处 - 使用goto的简化版本
        :param high: 高度，单位cm
        """
        logger.info(f"Setting altitude for device {self.target_channel_id} to {high}cm")
        self.goto(0, 0, high)

    def led(self, r: int, g: int, b: int):
        """
        设置无人机LED色彩 - MAV_CMD_EXT_DRONE_LIGHT_RGB (276)
        :param r: 红色值（0-255）
        :param g: 绿色值（0-255）
        :param b: 蓝色值（0-255）
        """
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        logger.info(f"Setting LED color for device {self.target_channel_id} to RGB({r}, {g}, {b})")
        # param1: R, param2: G, param3: B
        # param4: 呼吸灯模式, param5: 彩虹灯模式
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_LIGHT_RGB,
            param1=r,
            param2=g,
            param3=b,
            param4=0,
            param5=0
        )

    def bln(self, r: int, g: int, b: int):
        """
        设置无人机LED呼吸灯色彩 - MAV_CMD_EXT_DRONE_LIGHT_RGB (276)
        :param r: 红色值（0-255）
        :param g: 绿色值（0-255）
        :param b: 蓝色值（0-255）
        """
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        logger.info(f"Setting LED breathing mode for device {self.target_channel_id} to RGB({r}, {g}, {b})")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_LIGHT_RGB,
            param1=r,
            param2=g,
            param3=b,
            param4=1,  # 呼吸灯模式
            param5=0
        )

    def rainbow(self, r: int, g: int, b: int):
        """
        设置无人机LED彩虹色彩 - MAV_CMD_EXT_DRONE_LIGHT_RGB (276)
        :param r: 红色值（0-255）
        :param g: 绿色值（0-255）
        :param b: 蓝色值（0-255）
        """
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        logger.info(f"Setting LED rainbow mode for device {self.target_channel_id} to RGB({r}, {g}, {b})")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_LIGHT_RGB,
            param1=r,
            param2=g,
            param3=b,
            param4=0,
            param5=1  # 彩虹灯模式
        )

    def airplane_mode(self, mode: int):
        """
        设置无人机飞行模式 - MAV_CMD_EXT_DRONE_SET_MODE (277)
        :param mode: 1常规 2巡线 3跟随（通常情况下使用模式1）
        """
        mode = max(1, min(3, mode))
        logger.info(f"Setting airplane mode for device {self.target_channel_id} to {mode}")
        # param1: mode（1常规 2巡线 3跟随）
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_SET_MODE,
            param1=mode
        )

    def stop(self):
        """停桨 - 紧急停止"""
        logger.warning(f"Emergency stop for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0,      # disarm
            param2=21196   # 紧急停止魔术数字
        )

    def hover(self):
        """悬停 - 通过停止当前移动命令实现"""
        logger.info(f"Hovering device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_EXT_DRONE_MOVE,
            param1=1,  # 任意方向
            param2=0,  # 零距离
            param3=0,  # 零速度
            wait_for_finish=False
        )

    def flip_forward(self):
        """前翻 - 使用自定义命令"""
        logger.info(f"Flip forward for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_USER_4,
            param1=1  # 前翻
        )

    def flip_back(self):
        """后翻 - 使用自定义命令"""
        logger.info(f"Flip back for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_USER_4,
            param1=2  # 后翻
        )

    def flip_left(self):
        """左翻 - 使用自定义命令"""
        logger.info(f"Flip left for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_USER_4,
            param1=3  # 左翻
        )

    def flip_right(self):
        """右翻 - 使用自定义命令"""
        logger.info(f"Flip right for device {self.target_channel_id}")
        self._send_command_with_retry(
            command=mavlink2.MAV_CMD_USER_4,
            param1=4  # 右翻
        )
