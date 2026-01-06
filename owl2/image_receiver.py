import dataclasses
import functools
import threading
import time
from typing import Optional, Callable, Set

from .commonACFly import commonACFly_py3 as mavlink2


@dataclasses.dataclass
class ImageInfo:
    photo_id: int
    total_packets: int
    # dict[packet index , tuple(packet checksum, packet data)]
    # every packet is 64 bytes
    packet_cache: dict[int, tuple[int, bytes]] = dataclasses.field(default_factory=dict)
    # jpg formatted image data
    image_data: bytes = b""
    # 已请求重传的块索引，避免短时间内重复请求
    requested_packets: Set[int] = dataclasses.field(default_factory=set)
    # 最大已收到的块索引，用于乱序检测
    max_received_index: int = -1
    # 最后收到块的时间，用于超时检测
    last_packet_time: float = dataclasses.field(default_factory=time.time)
    # 传输开始时间，用于总超时检测
    start_time: float = dataclasses.field(default_factory=time.time)


# 	 <entry value="286" name="MAV_CMD_EXT_DRONE_TAKE_PHOTO" hasLocation="false" isDestination="false">
#       <description>xinguangfei ext take photo</description>
#       <param index="1" label="cmd" minValue="0" maxValue="1" default="0">cmd</param>
# 		<param index="2">Empty</param>
# 		<param index="3">Empty</param>
# 		<param index="4">Empty</param>
# 		<param index="5">Empty</param>
# 		<param index="6">Empty</param>
# 		<param index="7">timestemp</param>
#    </entry>
# 	<message id="804" name="PHOTO_TOTAL_INFORMATION_ADDR_XINGUANGFEI">
# 		<description>xinguangfei photo imformation</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t" name="total_num" instance="true">index</field>
# 	</message>
# 	<message id="805" name="PHOTO_TRANSMISSION_XINGUANGFEI">
# 		<description>xinguangfei photo data</description>
# 		<field type="uint8_t" name="index">index</field>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t[64]" name="data" invalid="UINT8_MAX">data</field>
# 		<field type="uint8_t" name="checksum" invalid="UINT8_MAX">checksum</field>
# 	</message>
# 	<message id="806" name="PHOTO_TOTAL_REQUEST_XINGUANGFEI">
# 		<description>xinguangfei photo request</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t" name="index" instance="true">index</field>
# 	</message>
# 	<message id="808" name="PHOTO_CLEAR_XINGUANGFEI">
# 		<description>xinguangfei photo finifsh</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id  0:clear all  >=1:Specified ID </field>
# 	</message>

# use 286 to take photo, then will receive a 804 message with total packets
# then receive multiple 805 messages with photo data packets
# after all packets received, combine them into image data
# if some packets missing, send 806 message to request missing packets
# after all packets received and image combined and no issue, send 808 message to remove received photo data in drone
# can first send a 808 message with id 0 to clear all previous photo data in drone
class ImageReceiver:
    airplane: 'AirplaneOwl02'
    # dict[photo_id, ImageInfo]
    image_table: dict[int, ImageInfo]
    # 超时检测定时器
    _timeout_timer: Optional[threading.Timer]
    # 包超时时间（秒），每个包约20ms，设置为300ms（约10个包的时间）可容忍一定波动
    PACKET_TIMEOUT: float = 0.3
    # 乱序容忍阈值：收到的包索引比期望索引大于此值时才认为丢包
    # 由于包间有业务数据干扰，允许轻微乱序（约3个包）
    OUT_OF_ORDER_THRESHOLD: int = 3
    # 总超时时间（秒），超过此时间强制结束（丢包情况下约5秒）
    TOTAL_TIMEOUT: float = 6.0
    # 完成回调
    _image_complete_callback: Optional[Callable[[int, bytes], None]]

    def __init__(self, airplane: 'AirplaneOwl02'):
        self.airplane = airplane
        self.image_table = {}
        self._timeout_timer = None
        self._image_complete_callback = None
        pass

    def _clean_image_table(self, photo_id: int = 0):
        """
        clean image_table after send 808
        :param photo_id:  0: clear all photos, >=1: clear specified photo id
        :return:
        """
        if photo_id != 0:
            if photo_id in self.image_table:
                del self.image_table[photo_id]
        else:
            self.image_table.clear()
        pass

    def send_msg_clear_photo(self, photo_id: int = 0):
        """
        send msg 808 to clear photo data in drone
        :param photo_id:  0: clear all photos, >=1: clear specified photo id
        :return:
        """
        self.airplane.send_command_with_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_CLEAR_XINGUANGFEI,
            param1=photo_id,
            ack_callback=lambda x: self._clean_image_table(photo_id),
        )
        pass

    def _send_msg_request_missing_packet(self, photo_id: int, packet_index: int):
        """
        send msg 806 to request missing photo packet
        :param photo_id:
        :param packet_index:
        :return:
        """
        self.airplane.send_command_with_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_TOTAL_REQUEST_XINGUANGFEI,
            param1=photo_id,
            param2=packet_index,
        )
        pass

    def on_image_info(self, message: mavlink2.MAVLink_photo_total_information_addr_xinguangfei_message):
        """
        call by AirplaneOwl02
        :param message:
        :return:
        """
        photo_id = message.photo_id
        total_packets = message.total_num
        if photo_id not in self.image_table:
            self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=total_packets)
        else:
            self.image_table[photo_id].total_packets = total_packets
        pass

    def on_image_packet(self, message: mavlink2.MAVLink_photo_transmission_xinguangfei_message):
        """
        call by AirplaneOwl02
        :param message:
        :return:
        """
        photo_id = message.photo_id
        packet_index = message.index
        packet_data = bytes(message.data)
        packet_checksum = message.checksum
        if photo_id not in self.image_table:
            # clean this image 808
            self.send_msg_clear_photo(photo_id=photo_id)
            return
        image_info = self.image_table[photo_id]
        image_info.packet_cache[packet_index] = (packet_checksum, packet_data)
        image_info.last_packet_time = time.time()

        # 如果这个包之前请求过重传，现在收到了，从requested中移除以便后续可以再次请求
        if packet_index in image_info.requested_packets:
            image_info.requested_packets.discard(packet_index)

        # 乱序检测：只有当跳跃超过阈值时才认为丢包
        expected_index = image_info.max_received_index + 1
        if packet_index > expected_index + self.OUT_OF_ORDER_THRESHOLD:
            # 检查从期望索引到当前索引之间缺失的块
            for missing_idx in range(expected_index, packet_index):
                if missing_idx not in image_info.packet_cache and missing_idx not in image_info.requested_packets:
                    # 请求重传该块
                    image_info.requested_packets.add(missing_idx)
                    self._send_msg_request_missing_packet(photo_id, missing_idx)

        # 更新最大已收到块索引
        if packet_index > image_info.max_received_index:
            image_info.max_received_index = packet_index

        # 重置超时定时器
        self._reset_timeout_timer(photo_id)

        # check if all packets received
        if image_info.total_packets > 0 and len(image_info.packet_cache) == image_info.total_packets:
            self._complete_image(photo_id)
        pass

    def _reset_timeout_timer(self, photo_id: int):
        """
        重置超时定时器，用于检测长时间未收到新块的情况
        """
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._timeout_timer = threading.Timer(self.PACKET_TIMEOUT, self._on_timeout, args=[photo_id])
        self._timeout_timer.daemon = True
        self._timeout_timer.start()

    def _on_timeout(self, photo_id: int):
        """
        超时处理：检查并请求所有缺失的块
        """
        if photo_id not in self.image_table:
            return
        image_info = self.image_table[photo_id]

        # 如果已经完成，不需要处理
        if image_info.image_data:
            return

        # 如果还不知道总块数，无法判断缺失
        if image_info.total_packets <= 0:
            return

        # 检查总超时
        elapsed = time.time() - image_info.start_time
        if elapsed > self.TOTAL_TIMEOUT:
            # 总超时，强制完成（可能图像不完整）
            self._complete_image(photo_id)
            return

        # 清除之前的请求标记，允许再次请求未收到的包
        image_info.requested_packets.clear()

        # 找出所有缺失的块并请求重传
        missing_count = 0
        for i in range(image_info.total_packets):
            if i not in image_info.packet_cache:
                image_info.requested_packets.add(i)
                self._send_msg_request_missing_packet(photo_id, i)
                missing_count += 1

        # 如果有缺失块，重新设置超时定时器等待重传
        if missing_count > 0:
            self._reset_timeout_timer(photo_id)
        # 如果所有块都收到，完成图像
        elif len(image_info.packet_cache) == image_info.total_packets:
            self._complete_image(photo_id)

    def _complete_image(self, photo_id: int):
        """
        完成图像接收：合并所有块并通知
        """
        if photo_id not in self.image_table:
            return
        image_info = self.image_table[photo_id]

        # 取消超时定时器
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        # 合并所有块
        image_data = bytearray()
        for i in range(image_info.total_packets):
            if i in image_info.packet_cache:
                checksum, data = image_info.packet_cache[i]
                image_data.extend(data)
            else:
                # 仍有缺失块，不应该到这里
                return
        image_info.image_data = bytes(image_data)

        # 通知完成
        if self._image_complete_callback is not None:
            self._image_complete_callback(photo_id, image_info.image_data)

        # 发送808清除无人机端数据
        self.send_msg_clear_photo(photo_id=photo_id)

    def set_image_complete_callback(self, callback: Optional[Callable[[int, bytes], None]]):
        """
        设置图像接收完成回调
        :param callback: function(photo_id: int, image_data: bytes)
        """
        self._image_complete_callback = callback

    def get_image(self, photo_id: int) -> bytes | bool:
        """
        get received image data by photo_id
        user call this after capture_image() to the image data
        :param photo_id:
        :return: image data in bytes, or True if image is still being received, or False if no such photo_id
        """
        if photo_id in self.image_table:
            image_info = self.image_table[photo_id]
            if image_info.image_data:
                return image_info.image_data
            else:
                return True
        return False

    def capture_image(self, callback: Optional[Callable[[int | None], None]] = None) -> int:
        """
        send command 286 to capture image
        user call this , then wait for callback to get photo_id
        :param callback: function(photo_id: int|None)
        :return: photo_id
        """
        # use current timestamp as photo_id
        import time
        photo_id = int(time.time()) % 256  # keep it in uint8 range
        self.airplane.send_command_with_retry(
            mavlink2.MAV_CMD_EXT_DRONE_TAKE_PHOTO,
            param1=0,
            ack_callback=functools.partial(self._when_capture_image_ack, callback=callback),
        )
        return photo_id

    def _when_capture_image_ack(self, cmd_status: 'CommandStatus',
                                callback: Optional[Callable[[int | None], None]] = None) -> None:
        """
        call in capture_image()
        :param cmd_status:
        :param callback:
        :return:
        """
        if cmd_status.is_finished or cmd_status.is_received:
            if cmd_status.ack_result_param2 is not None and cmd_status.ack_result_param2 != 0:
                photo_id = cmd_status.ack_result_param2
                self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=0)
                if callback is not None:
                    callback(photo_id)
            else:
                if callback is not None:
                    callback(None)
        pass
