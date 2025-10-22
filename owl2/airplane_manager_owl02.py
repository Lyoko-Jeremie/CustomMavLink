"""
无人机管理类，参考TypeScript的AirplaneManagerOwl02实现
"""
import asyncio
import serial
import time
import threading
from typing import Dict, Optional, Callable, Any
import logging
from pymavlink import mavutil

from .airplane_owl02 import AirplaneOwl02
from .custom_protocol_packet import PacketParser, send_mavlink_packet_by_custom_protocol

# 配置日志
logger = logging.getLogger(__name__)


class AirplaneManagerOwl02:
    """无人机管理类"""

    def __init__(self, serial_port: Optional[serial.Serial] = None):
        self.airplanes: Dict[int, AirplaneOwl02] = {}
        self.serial_port = serial_port
        self.packet_parser = PacketParser()
        self.is_init = False
        self.is_running = False

        # 心跳定时器相关
        self.heartbeat_timer = None
        self.heartbeat_interval = 1.0  # 1Hz
        self.heartbeat_enabled = True  # 心跳是否启用

        # 数据接收线程
        self.receive_thread = None
        self._stop_event = threading.Event()

        # 异步事件循环
        self.loop = None
        self.loop_thread = None

    def init(self):
        """初始化管理器"""
        if self.is_init:
            return
        self.is_init = True

        logger.info("Initializing AirplaneManagerOwl02")

        # 启动异步事件循环
        self._start_async_loop()

        # 启动心跳定时器
        self._start_heartbeat_timer()

        # 如果有串口，启动数据接收线程
        if self.serial_port:
            self._start_receive_thread()

        self.is_running = True
        logger.info("AirplaneManagerOwl02 initialized successfully")

    def _start_async_loop(self):
        """启动异步事件循环"""

        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()

        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()

        # 等待循环启动
        while self.loop is None:
            time.sleep(0.01)

    def _start_heartbeat_timer(self):
        """启动心跳定时器"""

        def heartbeat_task():
            while not self._stop_event.is_set():
                try:
                    # 向所有无人机发送心跳（同步方式）
                    self._send_heartbeat_to_all()
                except Exception as e:
                    logger.error(f"Error sending heartbeat: {e}")

                time.sleep(self.heartbeat_interval)

        self.heartbeat_timer = threading.Thread(target=heartbeat_task, daemon=True)
        self.heartbeat_timer.start()

    def _start_receive_thread(self):
        """启动数据接收线程"""

        def receive_task():
            while not self._stop_event.is_set() and self.serial_port and self.serial_port.is_open:
                try:
                    self._process_serial_data()
                except Exception as e:
                    logger.error(f"Error processing serial data: {e}")
                time.sleep(0.01)  # 避免CPU占用过高

        self.receive_thread = threading.Thread(target=receive_task, daemon=True)
        self.receive_thread.start()

    def _send_heartbeat_to_all(self):
        """向所有无人机发送心跳"""
        if not self.heartbeat_enabled:
            return  # 如果心跳未启用，直接返回

        for airplane_id, airplane in self.airplanes.items():
            try:
                airplane.send_heartbeat()
            except Exception as e:
                logger.error(f"Error sending heartbeat to airplane {airplane_id}: {e}")

    def enable_heartbeat(self):
        """启用心跳包发送"""
        self.heartbeat_enabled = True
        logger.info("Heartbeat enabled")

    def disable_heartbeat(self):
        """禁用心跳包发送"""
        self.heartbeat_enabled = False
        logger.info("Heartbeat disabled")

    def is_heartbeat_enabled(self) -> bool:
        """检查心跳是否启用"""
        return self.heartbeat_enabled

    def _process_serial_data(self):
        """处理串口数据"""
        if not self.serial_port or not self.serial_port.is_open:
            print('Serial port not open')
            return

        # 读取可用数据
        available_data = self.serial_port.read(self.serial_port.in_waiting or 1)
        if not available_data:
            print('available_data is empty')
            return

        # print('_process_serial_data', available_data)

        # 添加到解析器
        self.packet_parser.add_data(available_data)

        # 解析数据包
        packets = self.packet_parser.parse_packets()

        if len(packets) > 0:
            print('packets', packets)

        for packet_info, raw_data in packets:
            device_id = packet_info['device_id']
            payload = packet_info['payload']

            try:
                # 解析MavLink消息
                mavlink_messages = self._parse_mavlink_payload(payload)

                for msg in mavlink_messages:
                    # 异步处理消息
                    self._handle_mavlink_message(device_id, msg, payload)

            except Exception as e:
                logger.error(f"Error processing packet from device {device_id}: {e}")

    def _parse_mavlink_payload(self, payload: bytes) -> list:
        """解析MavLink载荷"""
        messages = []
        try:
            # 创建MavLink解析器
            mav = mavutil.mavlink.MAVLink(None)

            # 逐字节解析
            for byte in payload:
                msg = mav.parse_char(bytes([byte]))
                if msg:
                    messages.append(msg)

        except Exception as e:
            logger.error(f"Error parsing MavLink payload: {e}")

        return messages

    def _handle_mavlink_message(self, device_id: int, message: Any, raw_payload: bytes):
        """处理MavLink消息"""
        try:
            # 获取或创建无人机对象
            airplane = self.get_airplane(device_id)

            # 解析状态
            airplane.parse_state_from_mavlink(message, raw_payload)

            logger.debug(f"Processed message {message.get_msgId()} from device {device_id}")

        except Exception as e:
            logger.error(f"Error handling MavLink message from device {device_id}: {e}")

    def get_airplane(self, device_id: int) -> AirplaneOwl02:
        """获取或创建无人机对象"""
        if device_id < 0 or device_id > 15:
            raise ValueError("Device ID must be between 0 and 15")
        if device_id not in self.airplanes:
            # 创建新的无人机对象
            airplane = AirplaneOwl02(device_id, self)
            airplane.init()
            self.airplanes[device_id] = airplane
            logger.info(f"Created new airplane with ID: {device_id}")

        return self.airplanes[device_id]

    def send_msg(self, msg: Any, device_id: int):
        """发送消息给指定设备"""
        if not self.serial_port or not self.serial_port.is_open:
            logger.error("Serial port is not open")
            return False

        try:
            send_mavlink_packet_by_custom_protocol(self.serial_port, device_id, msg)
            logger.debug(f"Sent message to device {device_id}: {type(msg).__name__}")
            return True
        except Exception as e:
            logger.error(f"Error sending message to device {device_id}: {e}")
            return False

    def get_airplane_list(self) -> Dict[int, AirplaneOwl02]:
        """获取所有无人机列表"""
        return self.airplanes.copy()

    def get_airplane_by_id(self, device_id: int) -> Optional[AirplaneOwl02]:
        """根据ID获取无人机对象"""
        return self.airplanes.get(device_id)

    def remove_airplane(self, device_id: int) -> bool:
        """移除无人机"""
        if device_id in self.airplanes:
            del self.airplanes[device_id]
            logger.info(f"Removed airplane with ID: {device_id}")
            return True
        return False

    def set_serial_port(self, serial_port: serial.Serial):
        """设置串口"""
        old_port = self.serial_port
        self.serial_port = serial_port

        # 如果已经初始化且有旧串口，停止接收线程
        if self.is_init and old_port:
            self._stop_event.set()
            if self.receive_thread and self.receive_thread.is_alive():
                self.receive_thread.join(timeout=1.0)
            self._stop_event.clear()

        # 如果已经初始化且有新串口，启动接收线程
        if self.is_init and self.serial_port:
            self._start_receive_thread()

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        stats = {
            'airplane_count': len(self.airplanes),
            'is_running': self.is_running,
            'has_serial_port': self.serial_port is not None and self.serial_port.is_open,
            'airplanes': {}
        }

        for device_id, airplane in self.airplanes.items():
            airplane_stats = {
                'device_id': device_id,
                'is_armed': airplane.state.is_armed,
                'fly_mode': airplane.state.fly_mode.name if airplane.state.fly_mode else 'UNKNOWN',
                'is_landed': airplane.state.is_landed,
                'cached_packets_count': len(airplane.cached_packet_record),
                'gps_position': {
                    'lat': airplane.state.gps_position.lat,
                    'lon': airplane.state.gps_position.lon,
                    'alt': airplane.state.gps_position.alt,
                }
            }
            stats['airplanes'][device_id] = airplane_stats

        return stats

    def stop(self):
        """停止管理器"""
        if not self.is_running:
            return

        logger.info("Stopping AirplaneManagerOwl02")

        # 设置停止事件
        self._stop_event.set()

        # 等待线程结束
        if self.heartbeat_timer and self.heartbeat_timer.is_alive():
            self.heartbeat_timer.join(timeout=2.0)

        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)

        # 停止事件循环
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=2.0)

        # 关闭串口
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        self.is_running = False
        logger.info("AirplaneManagerOwl02 stopped")

    def __del__(self):
        """析构函数"""
        self.stop()


# 全局管理器实例
_global_manager = None

def get_airplane_manager_owl02() -> AirplaneManagerOwl02:
    """获取全局无人机管理器实例"""
    global _global_manager
    if _global_manager is None:
        _global_manager = AirplaneManagerOwl02()
        _global_manager.init()
    return _global_manager

# 便利函数
def create_manager_with_serial(port: str, baudrate: int = 115200, timeout: float = 1.0) -> AirplaneManagerOwl02:
    """创建带串口的管理器"""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        manager = AirplaneManagerOwl02(ser)
        return manager
    except serial.SerialException as e:
        logger.error(f"Failed to create serial connection: {e}")
        raise


def create_manager() -> AirplaneManagerOwl02:
    """创建不带串口的管理器"""
    return AirplaneManagerOwl02()
