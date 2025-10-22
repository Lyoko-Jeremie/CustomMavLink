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

from .commonACFly import commonACFly_py3 as mavlink2
from .custom_protocol_packet import send_mavlink_packet_raw, PacketParser


# TODO __mavlink_one_to_more_addr_xinguangfei_t

class AirplaneId:
    """
    __mavlink_one_to_more_addr_xinguangfei_t
    """

    def __init__(self, raw_pack: bytes, mtx_address: bytes, mrx_address_ack: bytes, mrx_address_p1: bytes):
        self.raw_pack = raw_pack    # __mavlink_one_to_more_addr_xinguangfei_t raw packet
        self.mtx_address = mtx_address  # type: bytes  # 5 bytes
        self.mrx_address_ack = mrx_address_ack  # type: bytes  # 5 bytes
        self.mrx_address_p1 = mrx_address_p1  # type: bytes  # 5 bytes
        # the hex str of mtx_address for display
        self.addr_hex_str = ''.join(f'{b:02X}' for b in mtx_address)
        pass

    pass


class PairManager:
    airplane_ids: dict[str, AirplaneId]
    paired_channels: dict[int, AirplaneId]

    serial_port_board: serial.Serial

    def __init__(self):
        self.airplane_ids = []
        self.paired_channels = {}

        pass

    def _get_airplane_id_from_serial(self, serial_port: str) -> AirplaneId:
        """
        从串口读取无人机ID
        :param serial_port: 串口号
        :return: 无人机ID
        """

        # TODO 先打开无人机端口，再发送请求报文，等待回复报文，解析出无人机ID并返回

        request_msg = mavlink2.MAVLink_one_to_more_addr_request_xinguangfei_message(
            request=1,
            reserved=[0] * 8,
        )

        # TODO 打开 serial_port

        # 发送 request_msg
        send_mavlink_packet_raw(serial_port, request_msg)

        # TODO 等待回复报文 MAVLINK_MSG_ID_ONE_TO_MORE_ADDR_XINGUANGFEI=801

        # TODO 解析回复报文，构造 AirplaneId 对象并返回
        # packet_parser = PacketParser() ??
        # receive_mavlink_packet ??

        pass

    def _set_airplane_id_to_channel(self, serial_port: str, channel: int, airplane_id: AirplaneId) -> bool:
        """
        将无人机ID写入地面板指定通道
        :param serial_port: 串口号
        :param channel: 通道号 0~15
        :param airplane_id: 无人机ID
        :return: 是否成功
        """

        # TODO 先打开地面板端口，再发送写入报文，等待确认报文

        # TODO 打开地面板端口

        # TODO 以 PROTOCOL_SETADDR_PAIR 模式发送 raw_pack 到地面板

        # TODO 等待确认报文 PROTOCOL_SETADDR_PAIR_ACK

        pass
