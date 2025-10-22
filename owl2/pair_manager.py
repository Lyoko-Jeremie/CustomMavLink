"""
# 配对管理器

使用方法：同时连接两个串口或先后连接两个串口，
从连接到无人机的串口上使用mavlink协议读取无人机的ID并保存到待匹配列表中，
然后从连接到地面板的串口上写入已读取的无人机ID到选定的0~15号通道中完成配对，
写入操作需要等待确认包返回。
同时附带一个从地面板读取当前设置的0~15号通道无人机ID的功能。

---

## 从连接到无人机的串口上使用mavlink协议读取无人机的ID

报文：MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_REQUEST_XINGUANGFEI=800
Typedefstruct __mavlink_one_to_more_addr_xinguangfei_t {
 uint8_t request;
uint8_t resever[8];
} mavlink_one_to_more_addr_xinguangfei_t;
上位机发送请求MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_REQUEST_XINGUANGFEI=800

飞控回复
MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801
报文：MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801
Typedefstruct __mavlink_one_to_more_addr_xinguangfei_t {
 uint8_t mtx_address[5];
 uint8_t mrx_address_ack[5];
 uint8_t mrx_address_p1[5];
} mavlink_one_to_more_addr_xinguangfei_t;

"""
import serial
import time

from .commonACFly import commonACFly_py3 as mavlink2
from .custom_protocol_packet import (
    send_mavlink_packet_raw,
    PacketParser,
    wrap_packet,
    PROTOCOL_SETADDR_PAIR,
    PROTOCOL_SETADDR_PAIR_ACK,
    PROTOCOL_SETADDR_PAIR_REQUEST,
    PROTOCOL_SETADDR_PAIR_REQUEST_ACK,
)


class AirplaneId:
    """
    __mavlink_one_to_more_addr_xinguangfei_t
    """

    def __init__(self, raw_pack: bytes, mtx_address: bytes, mrx_address_ack: bytes, mrx_address_p1: bytes):
        self.raw_pack = raw_pack  # __mavlink_one_to_more_addr_xinguangfei_t raw packet (mavlink message bytes)
        self.mtx_address = mtx_address  # type: bytes  # 5 bytes
        self.mrx_address_ack = mrx_address_ack  # type: bytes  # 5 bytes
        self.mrx_address_p1 = mrx_address_p1  # type: bytes  # 5 bytes
        # the hex str of mtx_address for display
        self.addr_hex_str = ''.join(f'{b:02X}' for b in mtx_address)
        pass

    pass


class PairManager:
    airplane_ids: list[AirplaneId]
    paired_channels: dict[int, AirplaneId]

    def __init__(self):
        self.airplane_ids = []
        self.paired_channels = {}
        pass

    @staticmethod
    def _receive_raw_mavlink_message(serial_port, timeout: float = 2.0, expected_msg_id: int = None):
        """
        接收原始 MAVLink 消息（不经过自定义协议封装）
        :param serial_port: 串口对象
        :param timeout: 超时时间（秒）
        :param expected_msg_id: 期望的消息ID，如果指定则只返回匹配的消息
        :return: MAVLink 消息对象，超时返回 None
        """
        mav_parser = mavlink2.MAVLink(None)
        start_time = time.time()

        while time.time() - start_time < timeout:
            if serial_port.in_waiting > 0:
                data = serial_port.read(1)
                msg = mav_parser.parse_char(data)

                if msg:
                    # 如果指定了期望的消息ID，检查是否匹配
                    if expected_msg_id is None or msg.get_msgId() == expected_msg_id:
                        return msg
                    # 如果不匹配，继续等待

            time.sleep(0.001)  # 短暂等待避免CPU占用过高

        return None

    def get_airplane_id_from_serial(self, serial_port: serial.Serial, timeout: float = 2.0) -> AirplaneId:
        """
        从串口读取无人机ID
        :param serial_port: 已打开的串口对象
        :param timeout: 超时时间（秒），默认2秒
        :return: 无人机ID
        """

        # 创建请求消息
        request_msg = mavlink2.MAVLink_one_to_more_addr_request_xinguangfei_message(
            request=1,
            reserved=[0] * 8,
        )

        # 清空接收缓冲区
        serial_port.reset_input_buffer()

        # 发送请求报文 (直接发送mavlink，不使用自定义协议封装)
        send_mavlink_packet_raw(serial_port, request_msg)

        # 等待回复报文 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801
        msg = self._receive_raw_mavlink_message(
            serial_port,
            timeout=timeout,
            expected_msg_id=mavlink2.MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI
        )

        if msg is None:
            raise TimeoutError(f"等待无人机ID回复超时 (>{timeout}秒)")

        # 解析回复报文，构造 AirplaneId 对象并返回
        # msg 有属性: mtx_address, mrx_address_ack, mrx_address_p1 (都是list或array)
        mtx_address = bytes(msg.mtx_address)
        mrx_address_ack = bytes(msg.mrx_address_ack)
        mrx_address_p1 = bytes(msg.mrx_address_p1)

        # TODO: 需要确认raw_pack是否应该是完整的mavlink消息字节
        # 这里暂时使用pack方法重新打包消息作为raw_pack
        mav_temp = mavlink2.MAVLink(None)
        raw_pack = msg.pack(mav_temp)

        airplane_id = AirplaneId(
            raw_pack=raw_pack,
            mtx_address=mtx_address,
            mrx_address_ack=mrx_address_ack,
            mrx_address_p1=mrx_address_p1
        )

        return airplane_id

    def set_airplane_id_to_channel(self, serial_port: serial.Serial, channel: int, airplane_id: AirplaneId,
                                   timeout: float = 2.0) -> bool:
        """
        将无人机ID写入地面板指定通道
        :param serial_port: 已打开的串口对象（地面板）
        :param channel: 通道号 0~15
        :param airplane_id: 无人机ID
        :param timeout: 超时时间（秒），默认2秒
        :return: 是否成功
        """

        if not (0 <= channel <= 15):
            raise ValueError(f"通道号必须在0-15之间，当前值: {channel}")

        # 清空接收缓冲区
        serial_port.reset_input_buffer()

        # 以 PROTOCOL_SETADDR_PAIR 模式发送 raw_pack 到地面板
        # 根据协议文档: 当协议识别码为 SETADDR_PAIR 时，payload 为 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801 原始包字节
        packet = wrap_packet(
            device_id=channel,  # 使用通道号作为设备ID
            data=airplane_id.raw_pack,  # mavlink 801消息的原始字节
            protocol_mode=PROTOCOL_SETADDR_PAIR
        )

        serial_port.write(packet)

        # 等待确认报文 PROTOCOL_SETADDR_PAIR_ACK
        # 根据协议文档: 当协议识别码为 SETADDR_PAIR_ACK 时，payload 为 uint8_t ack (0:失败，1:成功)
        packet_parser = PacketParser()
        start_time = time.time()

        while time.time() - start_time < timeout:
            if serial_port.in_waiting > 0:
                data = serial_port.read(serial_port.in_waiting)
                packet_parser.add_data(data)

                packets = packet_parser.parse_packets()
                for packet_info, raw_data in packets:
                    if packet_info['protocol_mode'] == PROTOCOL_SETADDR_PAIR_ACK:
                        # 检查ACK状态
                        payload = packet_info['payload']
                        if len(payload) > 0:
                            ack_status = payload[0]
                            if ack_status == 1:
                                print(f"配对成功: 通道{channel} <- {airplane_id.addr_hex_str}")
                                return True
                            else:
                                print(f"配对失败: 通道{channel}, ACK状态={ack_status}")
                                return False

            time.sleep(0.01)  # 短暂等待避免CPU占用过高

        # 超时未收到确认
        print(f"等待配对确认超时 (>{timeout}秒)")
        return False

    def get_all_channel_id_from_board(self, serial_port: serial.Serial, channel=0, timeout: float = 2.0) -> dict[
        int, AirplaneId]:
        """
        从地面板读取所有通道的无人机ID
        :param serial_port: 已打开的串口对象（地面板）
        :param timeout: 超时时间（秒），默认2秒
        :return: 字典，键为通道号(0-15)，值为AirplaneId对象
        """

        # 清空接收缓冲区和已配对通道记录
        serial_port.reset_input_buffer()
        self.paired_channels.clear()

        # 发送请求包，要求地面板返回当前所有通道的无人机ID
        request_packet = wrap_packet(
            device_id=channel,  # 设备ID设为0，表示请求所有通道信息
            data=b'',  # 无需附加数据
            protocol_mode=PROTOCOL_SETADDR_PAIR_REQUEST
        )

        serial_port.write(request_packet)

        # 等待回复
        # 根据协议文档：16个通道共4个包，每个包会重发3次以确保数据正确接收
        # 每个包的payload包含4个结构体，每个结构体16字节（1+5+5+5）
        packet_parser = PacketParser()
        start_time = time.time()
        received_packets = set()  # 用于去重（因为每个包会重发3次）

        while time.time() - start_time < timeout:
            if serial_port.in_waiting > 0:
                data = serial_port.read(serial_port.in_waiting)
                packet_parser.add_data(data)

                packets = packet_parser.parse_packets()
                for packet_info, raw_data in packets:
                    if packet_info['protocol_mode'] == PROTOCOL_SETADDR_PAIR_REQUEST_ACK:
                        payload = packet_info['payload']

                        # 每个payload包含多个16字节的结构体
                        # 结构体格式: uint8_t id + uint8_t mtx_address[5] + uint8_t mrx_address_ack[5] + uint8_t mrx_address_p1[5]
                        struct_size = 16  # 1 + 5 + 5 + 5
                        num_structs = len(payload) // struct_size

                        for i in range(num_structs):
                            offset = i * struct_size
                            struct_data = payload[offset:offset + struct_size]

                            if len(struct_data) < struct_size:
                                continue

                            # 解析结构体
                            channel_id = struct_data[0]
                            mtx_address = struct_data[1:6]
                            mrx_address_ack = struct_data[6:11]
                            mrx_address_p1 = struct_data[11:16]

                            # 检查通道号是否有效
                            if not (0 <= channel_id <= 15):
                                continue

                            # 使用通道ID和地址创建唯一标识符用于去重
                            channel_key = (channel_id, bytes(mtx_address))
                            if channel_key in received_packets:
                                continue  # 跳过重复的包

                            received_packets.add(channel_key)

                            # 创建完整的mavlink 801消息格式的raw_pack
                            # 这里重构一个简化的包用于存储
                            mav_temp = mavlink2.MAVLink(None)
                            temp_msg = mavlink2.MAVLink_one_to_more_addr_xinguangfei_message(
                                mtx_address=list(mtx_address),
                                mrx_address_ack=list(mrx_address_ack),
                                mrx_address_p1=list(mrx_address_p1)
                            )
                            raw_pack = temp_msg.pack(mav_temp)

                            # 构造 AirplaneId 对象并保存到 paired_channels
                            airplane_id = AirplaneId(
                                raw_pack=raw_pack,
                                mtx_address=bytes(mtx_address),
                                mrx_address_ack=bytes(mrx_address_ack),
                                mrx_address_p1=bytes(mrx_address_p1)
                            )

                            self.paired_channels[channel_id] = airplane_id
                            print(f"读取通道{channel_id}配对信息: {airplane_id.addr_hex_str}")

            time.sleep(0.01)  # 短暂等待避免CPU占用过高

        print(f"成功读取 {len(self.paired_channels)} 个通道的配对信息")
        return self.paired_channels
