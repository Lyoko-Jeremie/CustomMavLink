"""
数据包封装和解析工具
包含数据包的封装、解析和发送功能
"""
import struct
from commonACFly import commonACFly_py3 as mavlink2

# 封装包
# 帧头1	帧头2	ID	                    数据长度	    PLAYLOAD(data)	    uint8_t校验和	帧尾
# 0xAA	0xBB	0-15（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
#
# 备注：id为0-15个天空端的设备ID
#       playload为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到playload
HEADER1 = 0xAA
HEADER2 = 0xBB
TAIL = 0xCC
MAX_PAYLOAD_SIZE = 58


def wrap_packet(device_id: int, data: bytes) -> bytes:
    """
    封装数据包
    格式: 0xAA 0xBB ID 数据长度 PAYLOAD 校验和 0xCC
    """

    # print('wrap_packet data', data)

    if not (0 <= device_id <= 15):
        raise ValueError("Device ID must be between 0 and 15")

    if len(data) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"Data size {len(data)} exceeds maximum {MAX_PAYLOAD_SIZE}")

    data_length = len(data)

    # 构建包体（不包括校验和和帧尾）
    packet_body = struct.pack('BBB', HEADER1, HEADER2, device_id) + struct.pack('B', data_length) + data

    # 计算校验和（对包体所有字节求和）
    checksum = sum(packet_body) & 0xFF

    # 完整数据包
    packet = packet_body + struct.pack('BB', checksum, TAIL)

    # print('wrap_packet packet', packet)

    return packet


# 解析包
# 帧头1	帧头2	ID	                    数据长度	    PLAYLOAD(data)	    uint8_t校验和	帧尾
# 0xAA	0xBB	0-15（用于判断设备号） 	max值（58）	max值（58个字节）		checksum        0xCC
#
# 备注：id为0-15个天空端的设备ID
#       playload为天空端设备回传的信息或者地面站发送的cmd，地面站与天空端之间采用mavlink数据传输。先将基本数据打包成mavlink，打包后的mavlink数据放到playload
class PacketParser:
    """数据包解析器，支持缓存和包切割"""

    def __init__(self):
        self.buffer = bytearray()

    def add_data(self, data: bytes):
        """添加新收到的数据到缓存"""
        self.buffer.extend(data)

    def parse_packets(self):
        """从缓存中解析出完整的数据包"""
        packets = []

        # print('parse_packets buffer', self.buffer)

        while len(self.buffer) >= 6:  # 最小包长度：头(2) + ID(1) + 长度(1) + 校验(1) + 尾(1)
            # 查找包头
            header_pos = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == HEADER1 and self.buffer[i + 1] == HEADER2:
                    header_pos = i
                    break

            if header_pos == -1:
                # 没找到包头，保留最后一个字节（如果是0xAA可能是包头的一部分）
                if len(self.buffer) > 0 and self.buffer[-1] == HEADER1:
                    self.buffer = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break

            # 移除包头前的无效数据
            if header_pos > 0:
                self.buffer = self.buffer[header_pos:]

            # 检查是否有足够的数据解析包头信息
            if len(self.buffer) < 4:
                break

            # 解析包头信息
            device_id = self.buffer[2]
            data_length = self.buffer[3]

            # 计算完整包的长度
            total_length = 4 + data_length + 2  # 头(2) + ID(1) + 长度(1) + 数据(data_length) + 校验(1) + 尾(1)

            # 检查是否有完整的包
            if len(self.buffer) < total_length:
                break

            # 提取完整包
            packet_data = bytes(self.buffer[:total_length])

            # 验证包的完整性
            try:
                parsed_packet = self._parse_single_packet(packet_data)
                if parsed_packet:
                    packets.append(parsed_packet)
                # 从缓存中移除已处理的包
                self.buffer = self.buffer[total_length:]
            except ValueError as e:
                print(f"Package parsing error: {e}")
                # 解析失败，跳过当前包头，继续查找下一个包头
                self.buffer = self.buffer[2:]

        return packets

    def _parse_single_packet(self, packet: bytes) -> dict:
        """解析单个数据包"""
        if len(packet) < 6:
            raise ValueError("Packet too short")

        # 检查包头
        if packet[0] != HEADER1 or packet[1] != HEADER2:
            raise ValueError("Invalid header")

        # 解析字段
        device_id = packet[2]
        data_length = packet[3]

        if not (0 <= device_id <= 15):
            raise ValueError(f"Invalid device ID: {device_id}")

        if data_length > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Data length {data_length} exceeds maximum {MAX_PAYLOAD_SIZE}")

        # 检查包长度
        expected_length = 4 + data_length + 2
        if len(packet) != expected_length:
            raise ValueError(f"Packet length mismatch: expected {expected_length}, got {len(packet)}")

        # 提取数据
        payload = packet[4:4 + data_length]
        checksum = packet[4 + data_length]
        tail = packet[4 + data_length + 1]

        # 验证帧尾
        if tail != TAIL:
            raise ValueError(f"Invalid tail: expected {TAIL}, got {tail}")

        # 验证校验和
        packet_body = packet[:4 + data_length]
        calculated_checksum = sum(packet_body) & 0xFF
        if checksum != calculated_checksum:
            raise ValueError(f"Checksum mismatch: expected {calculated_checksum}, got {checksum}")

        return {
            'device_id': device_id,
            'payload': payload
        }


def send_mavlink_packet(serial_port, device_id: int, mav_msg):
    """发送MavLink数据包"""
    # 创建MavLink对象来序列化消息
    print('send_mavlink_packet device_id', device_id, mav_msg)
    mav = mavlink2.MAVLink(None)
    mav_bytes = mav_msg.pack(mav)
    wrapped = wrap_packet(device_id, mav_bytes)
    serial_port.write(wrapped)


def receive_mavlink_packet(serial_port, packet_parser=None):
    """
    接收MavLink数据包，支持数据缓存和包切割

    Args:
        serial_port: 串口对象
        packet_parser: PacketParser实例，如果为None则创建新实例

    Returns:
        包含device_id和mavlink_msg的字典，或None
    """
    if packet_parser is None:
        packet_parser = PacketParser()

    # 读取可用的数据
    available_data = serial_port.read(serial_port.in_waiting or 1)
    if available_data:
        packet_parser.add_data(available_data)

    # 尝试解析数据包
    packets = packet_parser.parse_packets()

    if packets:
        # 返回第一个解析到的包
        packet_info = packets[0]
        payload = packet_info['payload']
        device_id = packet_info['device_id']

        # 解析MavLink消息
        try:
            # 创建MavLink解析器
            mav = mavlink2.MAVLink(None)
            msgs = []

            # 逐字节解析MavLink消息
            for byte in payload:
                msg = mav.parse_char(bytes([byte]))
                if msg:
                    msgs.append(msg)

            if msgs:
                return {
                    'device_id': device_id,
                    'mavlink_msg': msgs[0]  # 返回第一个解析到的消息
                }
        except Exception as e:
            print(f"MavLink parsing error: {e}")
            return {
                'device_id': device_id,
                'raw_payload': payload
            }

    return None
