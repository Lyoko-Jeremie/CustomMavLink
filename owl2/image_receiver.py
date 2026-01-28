import dataclasses
import threading
import time
from collections import deque
from typing import Optional, Callable, Set

from .commonACFly import commonACFly_py3 as mavlink2


@dataclasses.dataclass
class PendingCaptureRequest:
    """拍照请求的 pending 记录"""
    callback: Optional[Callable[[int | None], None]]
    expire_time: float  # 超时时间点（绝对时间）


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


#               拍照请求，无人机收到后拍照并存储照片数据，返回照片ID（uint8）
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
#               传输开始时的第一个包，照片信息，包括照片ID和总包数
# 	<message id="804" name="PHOTO_TOTAL_INFORMATION_ADDR_XINGUANGFEI">
# 		<description>xinguangfei photo imformation</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t" name="total_num" instance="true">index</field>
# 	</message>
#               数据体传输包，包含照片ID，包索引，数据和校验和
# 	<message id="805" name="PHOTO_TRANSMISSION_XINGUANGFEI">
# 		<description>xinguangfei photo data</description>
# 		<field type="uint8_t" name="index">index</field>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t[64]" name="data" invalid="UINT8_MAX">data</field>
# 		<field type="uint8_t" name="checksum" invalid="UINT8_MAX">checksum</field>
# 	</message>
#               请求缺失的包
# 	<message id="806" name="PHOTO_TOTAL_REQUEST_XINGUANGFEI">
# 		<description>xinguangfei photo request</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t" name="index" instance="true">index</field>
# 	</message>
#               拍照请求 MAV_CMD_EXT_DRONE_TAKE_PHOTO 的响应，包含照片ID和是否成功
# 	<message id="807" name="TAKE_PHOTO_ACK_XINGUANGFEI">
# 		<description>xinguangfei photo request</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id</field>
# 		<field type="uint8_t" name="result" instance="true">result</field>
# 	</message>
#               当接受完毕时，清除无人机端数据。或者在开始前清除之前的数据(photo_id=0)
# 	<message id="808" name="PHOTO_CLEAR_XINGUANGFEI">
# 		<description>xinguangfei photo finifsh</description>
# 		<field type="uint8_t" name="photo_id" instance="true">id  0:clear all  >=1:Specified ID </field>
# 	</message>


# 286 拍照 -> 807 确认拍照收到 -> 804 照片信息 -> 806 请求照片数据 -> 805 照片数据包 -> 808 清除照片数据
# use 286 to take photo, will receive 807 with photo_id to know ok or not
# then will receive a 804 message with total packets
# send a 806 with photo id and packet index 255 to start receiving photo data
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
    # 拍照请求的 pending 队列（FIFO），带超时时间
    _pending_capture_requests: deque[PendingCaptureRequest]
    # 超时检查定时器
    _capture_timeout_timer: Optional[threading.Timer]
    # 拍照请求超时时间（秒），考虑到 286->807 延迟约 2-3 秒，设置为 5 秒
    CAPTURE_TIMEOUT: float = 5.0

    def __init__(self, airplane: 'AirplaneOwl02'):
        self.airplane = airplane
        self.image_table = {}
        self._timeout_timer = None
        self._image_complete_callback = None
        self._pending_capture_requests = deque()
        self._capture_timeout_timer = None
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
        # TODO need ack ?
        self.airplane.send_command_without_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_CLEAR_XINGUANGFEI,
            param1=photo_id,
            ack_callback=lambda x: self._clean_image_table(photo_id),
        )
        print('ImageReceiver.send_msg_clear_photo: photo_id=[{}]'.format(photo_id))
        pass

    def _send_start_receive_photo(self, photo_id: int):
        """
        send msg 806 to start receiving photo data
        :param photo_id:
        :return:
        """
        self.airplane.send_command_without_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_TOTAL_REQUEST_XINGUANGFEI,
            param1=photo_id,
            param2=255,  # 255 means start receiving all packets
        )
        print('ImageReceiver._send_start_receive_photo: photo_id=[{}]'.format(photo_id))
        pass

    def _send_msg_request_missing_packet(self, photo_id: int, packet_index: int):
        """
        send msg 806 to request missing photo packet
        :param photo_id:
        :param packet_index:
        :return:
        """
        self.airplane.send_command_without_retry(
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
        print('ImageReceiver.on_image_info: photo_id=[{}], total_packets=[{}]'.format(photo_id, total_packets))
        if photo_id not in self.image_table:
            self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=total_packets)
        else:
            self.image_table[photo_id].total_packets = total_packets
            pass
        # send msg 806 to start receiving photo data
        self._send_start_receive_photo(photo_id=photo_id)
        pass

    def on_image_packet(self, message: mavlink2.MAVLink_photo_transmission_xinguangfei_message):
        """
        call by AirplaneOwl02
        :param message:
        :return:
        """
        photo_id = message.photo_id
        packet_index = message.index
        print('ImageReceiver.on_image_packet: photo_id=[{}], packet_index=[{}]'.format(photo_id, packet_index))
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

    def capture_image(self, callback: Optional[Callable[[int | None], None]] = None):
        """
        send command 286 to capture image
        用户调用此函数发送拍照命令，实际的 photo_id 会在收到 807 消息后通过 callback 返回

        注意：
        - 286 拍照操作不是幂等的，不能重发
        - 拍照操作有时效性，应该立即发送
        - 从发送 286 到收到 807 约有 2-3 秒延迟
        - 如果超时（5秒）未收到 807，认为请求失败
        - 允许多个 pending 请求并行等待，按 FIFO 匹配 807 响应

        :param callback: function(photo_id: int|None) - 成功时传入 photo_id，失败/超时时传入 None
        """
        # 创建 pending 请求记录
        pending_request = PendingCaptureRequest(
            callback=callback,
            expire_time=time.time() + self.CAPTURE_TIMEOUT,
        )

        # 将请求加入 pending 队列
        self._pending_capture_requests.append(pending_request)

        # 启动超时检查定时器（如果尚未运行）
        self._start_capture_timeout_checker()

        # 使用 send_command_without_retry 发送拍照命令
        self.airplane.send_command_without_retry(
            mavlink2.MAV_CMD_EXT_DRONE_TAKE_PHOTO,
            param1=0,
        )
        pending_count = len(self._pending_capture_requests)
        print(f'ImageReceiver.capture_image: sent take photo command, pending_count=[{pending_count}], waiting for ack')
        pass

    def _start_capture_timeout_checker(self):
        """启动超时检查定时器（如果队列非空且定时器未运行）"""
        if self._capture_timeout_timer is not None:
            return  # 定时器已在运行
        if not self._pending_capture_requests:
            return  # 队列为空，无需启动

        # 计算下一个最早超时的时间
        next_expire = self._pending_capture_requests[0].expire_time
        delay = max(0.1, next_expire - time.time())  # 至少 0.1 秒后检查

        self._capture_timeout_timer = threading.Timer(delay, self._check_capture_timeouts)
        self._capture_timeout_timer.daemon = True
        self._capture_timeout_timer.start()

    def _check_capture_timeouts(self):
        """检查并处理超时的拍照请求"""
        self._capture_timeout_timer = None
        current_time = time.time()

        # 从队列头部移除所有超时的请求
        while self._pending_capture_requests:
            request = self._pending_capture_requests[0]
            if request.expire_time <= current_time:
                # 请求已超时
                self._pending_capture_requests.popleft()
                print(f'ImageReceiver._check_capture_timeouts: capture request timed out')
                if request.callback is not None:
                    request.callback(None)
            else:
                # 队列头部未超时，后面的也不会超时（FIFO），退出循环
                break

        # 如果队列仍有请求，继续定时检查
        self._start_capture_timeout_checker()

    def on_take_photo_ack(self, message: mavlink2.MAVLink_take_photo_ack_xinguangfei_message):
        """
        call by AirplaneOwl02
        收到 807 消息后，按 FIFO 顺序匹配 pending 请求并调用回调
        如果队列为空，发送 808(photo_id=0) 清除远端所有图片数据

        :param message:
        :return:
        """
        photo_id = message.photo_id
        result = message.result
        print(f'ImageReceiver.on_take_photo_ack: photo_id=[{photo_id}], result=[{result}]')

        # 检查是否有等待中的回调
        if self._pending_capture_requests:
            # 从队列头部取出请求（FIFO）
            pending_request = self._pending_capture_requests.popleft()
            print(f'ImageReceiver.on_take_photo_ack: matched pending request')

            if result == 0:
                # 拍照成功，初始化 image_table 条目
                if photo_id not in self.image_table:
                    self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=0)
                # 调用 callback，传入 photo_id
                if pending_request.callback is not None:
                    pending_request.callback(photo_id)
            else:
                # 拍照失败，调用 callback，传入 None
                if pending_request.callback is not None:
                    pending_request.callback(None)
        else:
            # 队列为空，没有等待的回调，发送 808(photo_id=0) 清除远端所有图片数据
            print('ImageReceiver.on_take_photo_ack: no pending request, clearing remote photo data')
            self.send_msg_clear_photo(photo_id=0)
        pass
