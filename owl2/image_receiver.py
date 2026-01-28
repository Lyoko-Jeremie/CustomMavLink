"""
图像接收器模块 (Image Receiver Module)

本模块实现了无人机图像传输的接收端逻辑，基于MAVLink协议进行通信。
主要功能包括：
1. 发送拍照命令并等待确认
2. 接收并缓存图像数据包
3. 检测丢包并请求重传
4. 组装完整图像数据
5. 清除已传输完成的图像

通信流程:
    发送286拍照命令 -> 收到807拍照确认 -> 收到804图像信息 
    -> 发送806开始传输 -> 收到多个805数据包 -> 发送808清除数据

相关MAVLink消息:
    - MAV_CMD_EXT_DRONE_TAKE_PHOTO (286): 拍照命令
    - PHOTO_TOTAL_INFORMATION_ADDR_XINGUANGFEI (804): 图像信息（照片ID、总包数）
    - PHOTO_TRANSMISSION_XINGUANGFEI (805): 图像数据包
    - PHOTO_TOTAL_REQUEST_XINGUANGFEI (806): 请求数据包/开始传输
    - TAKE_PHOTO_ACK_XINGUANGFEI (807): 拍照确认响应
    - PHOTO_CLEAR_XINGUANGFEI (808): 清除图像数据
"""

import dataclasses
import threading
import time
from collections import deque
from typing import Optional, Callable, Set

from .commonACFly import commonACFly_py3 as mavlink2


@dataclasses.dataclass
class PendingCaptureRequest:
    """
    拍照请求的等待记录
    
    用于追踪已发送但尚未收到响应的拍照请求。
    采用FIFO队列管理多个并发请求。
    
    Attributes:
        callback: 拍照完成后的回调函数，参数为photo_id（成功）或None（失败/超时）
        expire_time: 请求过期的绝对时间戳，超过此时间视为超时
    """
    callback: Optional[Callable[[int | None], None]]
    expire_time: float  # 超时时间点（绝对时间）


@dataclasses.dataclass
class ImageInfo:
    """
    图像信息数据类
    
    存储单张图像的传输状态和数据，包括接收进度、缓存数据、超时状态等。
    
    Attributes:
        photo_id: 图像的唯一标识符（0-255）
        total_packets: 图像被分割成的数据包总数
        packet_cache: 已接收数据包的缓存，格式为 {包索引: (校验和, 64字节数据)}
        image_data: 组装完成的JPEG格式图像数据
        requested_packets: 已请求重传的包索引集合，避免短时间内重复请求同一个包
        max_received_index: 已收到的最大包索引，用于乱序检测
        last_packet_time: 最后一次收到数据包的时间戳，用于单包超时检测
        start_time: 传输开始时间，用于总超时检测
    """
    photo_id: int                   # 照片ID，由无人机分配
    total_packets: int              # 数据包总数，从804消息获取
    # dict[packet index , tuple(packet checksum, packet data)]
    # 数据包缓存字典，键为包索引，值为(校验和, 64字节数据)的元组
    # 每个数据包固定64字节
    packet_cache: dict[int, tuple[int, bytes]] = dataclasses.field(default_factory=dict)
    # JPEG格式的图像数据，所有包组装完成后填充
    image_data: bytes = b""
    # 已请求重传的块索引集合，避免短时间内重复请求同一个丢失的包
    requested_packets: Set[int] = dataclasses.field(default_factory=set)
    # 最大已收到的块索引，用于乱序检测判断是否存在丢包
    max_received_index: int = -1
    # 最后收到数据包的时间戳，用于检测传输是否停滞
    last_packet_time: float = dataclasses.field(default_factory=time.time)
    # 传输开始时间，用于总超时检测，防止传输无限等待
    start_time: float = dataclasses.field(default_factory=time.time)


# ==================== MAVLink 协议定义 ====================
# 以下是本模块使用的MAVLink消息和命令的XML定义参考
#
# ------- 拍照命令 (286) -------
# 无人机收到后拍照并存储照片数据，返回照片ID（uint8）
# <entry value="286" name="MAV_CMD_EXT_DRONE_TAKE_PHOTO" hasLocation="false" isDestination="false">
#     <description>xinguangfei ext take photo</description>
#     <param index="1" label="cmd" minValue="0" maxValue="1" default="0">cmd</param>
#     <param index="2">Empty</param>
#     <param index="3">Empty</param>
#     <param index="4">Empty</param>
#     <param index="5">Empty</param>
#     <param index="6">Empty</param>
#     <param index="7">timestamp</param>
# </entry>
#
# ------- 图像信息消息 (804) -------
# 传输开始时的第一个包，包含照片ID和总包数
# <message id="804" name="PHOTO_TOTAL_INFORMATION_ADDR_XINGUANGFEI">
#     <description>xinguangfei photo information</description>
#     <field type="uint8_t" name="photo_id" instance="true">照片ID</field>
#     <field type="uint8_t" name="total_num" instance="true">数据包总数</field>
# </message>
#
# ------- 图像数据包消息 (805) -------
# 数据体传输包，包含照片ID、包索引、64字节数据和校验和
# <message id="805" name="PHOTO_TRANSMISSION_XINGUANGFEI">
#     <description>xinguangfei photo data</description>
#     <field type="uint8_t" name="index">包索引</field>
#     <field type="uint8_t" name="photo_id" instance="true">照片ID</field>
#     <field type="uint8_t[64]" name="data" invalid="UINT8_MAX">64字节图像数据</field>
#     <field type="uint8_t" name="checksum" invalid="UINT8_MAX">校验和</field>
# </message>
#
# ------- 请求数据包消息 (806) -------
# 请求缺失的包或开始传输（index=255表示开始接收所有包）
# <message id="806" name="PHOTO_TOTAL_REQUEST_XINGUANGFEI">
#     <description>xinguangfei photo request</description>
#     <field type="uint8_t" name="photo_id" instance="true">照片ID</field>
#     <field type="uint8_t" name="index" instance="true">包索引（255=开始传输）</field>
# </message>
#
# ------- 拍照确认消息 (807) -------
# 拍照请求 MAV_CMD_EXT_DRONE_TAKE_PHOTO 的响应，包含照片ID和是否成功
# <message id="807" name="TAKE_PHOTO_ACK_XINGUANGFEI">
#     <description>xinguangfei photo request</description>
#     <field type="uint8_t" name="photo_id" instance="true">照片ID</field>
#     <field type="uint8_t" name="result" instance="true">结果（0=成功）</field>
# </message>
#
# ------- 清除图像消息 (808) -------
# 接收完毕后清除无人机端数据，或在开始前清除之前的数据(photo_id=0清除所有)
# <message id="808" name="PHOTO_CLEAR_XINGUANGFEI">
#     <description>xinguangfei photo finish</description>
#     <field type="uint8_t" name="photo_id" instance="true">照片ID（0=清除所有，>=1=指定ID）</field>
# </message>
# ==================== 协议定义结束 ====================


# ==================== 图像传输流程说明 ====================
# 完整流程: 286拍照 -> 807确认 -> 804图像信息 -> 806请求数据 -> 805数据包 -> 808清除
#
# 详细步骤:
# 1. 发送286命令触发拍照，无人机拍照并存储
# 2. 收到807消息，获取photo_id确认拍照成功
# 3. 收到804消息，获取图像的数据包总数
# 4. 发送806消息（index=255）开始接收数据
# 5. 持续接收805消息，缓存所有数据包
# 6. 如果检测到丢包，发送806消息请求重传特定包
# 7. 所有包接收完成后，组装成完整图像
# 8. 发送808消息清除无人机端的图像数据
#
# 注意: 可以先发送808(id=0)清除所有历史数据，再开始新的传输
# ==================== 流程说明结束 ====================


class ImageReceiver:
    """
    图像接收器类
    
    负责与无人机进行图像传输通信，处理拍照命令、数据包接收、
    丢包重传、超时处理等功能。
    
    使用方法:
        1. 创建实例: receiver = ImageReceiver(airplane)
        2. 设置完成回调: receiver.set_image_complete_callback(callback)
        3. 发送拍照命令: receiver.capture_image(callback)
        4. 等待回调通知获取photo_id
        5. 使用get_image(photo_id)获取图像数据
    
    Attributes:
        airplane: 关联的无人机实例（AirplaneOwl02类型）
        image_table: 图像信息表，键为photo_id，值为ImageInfo对象
        PACKET_TIMEOUT: 单包超时时间（秒），超时后请求重传
        OUT_OF_ORDER_THRESHOLD: 乱序容忍阈值，允许一定程度的包乱序
        TOTAL_TIMEOUT: 总传输超时时间（秒），超时后强制完成
        CAPTURE_TIMEOUT: 拍照命令超时时间（秒）
    """
    
    # 类型注解：关联的无人机实例
    airplane: 'AirplaneOwl02'
    # 图像信息表：photo_id -> ImageInfo 的映射
    image_table: dict[int, ImageInfo]
    # 数据包超时检测定时器
    _timeout_timer: Optional[threading.Timer]
    
    # ==================== 超时和阈值配置 ====================
    # 单包超时时间（秒）
    # 每个包传输约20ms，设置300ms可容忍约15个包的延迟波动
    PACKET_TIMEOUT: float = 0.3
    
    # 乱序容忍阈值
    # 当收到的包索引比期望索引大于此阈值时，才认为中间的包丢失
    # 由于传输过程中可能有其他业务数据穿插，允许轻微乱序
    OUT_OF_ORDER_THRESHOLD: int = 3
    
    # 总超时时间（秒）
    # 整个图像传输的最大等待时间，超时后强制完成（可能图像不完整）
    TOTAL_TIMEOUT: float = 6.0
    
    # 图像接收完成回调函数：function(photo_id: int, image_data: bytes)
    _image_complete_callback: Optional[Callable[[int, bytes], None]]
    
    # 拍照请求等待队列（FIFO顺序），用于匹配807响应
    _pending_capture_requests: deque[PendingCaptureRequest]
    
    # 拍照请求超时检查定时器
    _capture_timeout_timer: Optional[threading.Timer]
    
    # 拍照请求超时时间（秒）
    # 从发送286命令到收到807响应的延迟约2-3秒，设置20秒留有余量
    CAPTURE_TIMEOUT: float = 20.0

    def __init__(self, airplane: 'AirplaneOwl02'):
        """
        初始化图像接收器
        
        Args:
            airplane: 关联的无人机实例，用于发送MAVLink消息
        """
        self.airplane = airplane              # 保存无人机引用
        self.image_table = {}                 # 初始化图像信息表
        self._timeout_timer = None            # 数据包超时定时器
        self._image_complete_callback = None  # 完成回调函数
        self._pending_capture_requests = deque()  # 拍照请求队列
        self._capture_timeout_timer = None    # 拍照超时定时器
        pass

    def _clean_image_table(self, photo_id: int = 0):
        """
        清理本地图像缓存表
        
        在发送808清除消息后调用，同步清理本地的图像数据缓存。
        
        Args:
            photo_id: 要清除的照片ID
                      0: 清除所有照片数据
                      >=1: 清除指定ID的照片数据
        """
        if photo_id != 0:
            # 清除指定ID的图像
            if photo_id in self.image_table:
                del self.image_table[photo_id]
        else:
            # 清除所有图像
            self.image_table.clear()
        pass

    def send_msg_clear_photo(self, photo_id: int = 0):
        """
        发送808消息清除无人机端的图像数据
        
        在图像传输完成后调用，释放无人机端的存储空间。
        也可以在开始新传输前调用（photo_id=0）清除所有历史数据。
        
        Args:
            photo_id: 要清除的照片ID
                      0: 清除所有照片数据
                      >=1: 清除指定ID的照片数据
        
        Note:
            消息发送成功后会通过回调清理本地缓存
        """
        # TODO: 是否需要确认机制？当前使用回调但无重试
        self.airplane.send_command_without_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_CLEAR_XINGUANGFEI,
            param1=photo_id,
            ack_callback=lambda x: self._clean_image_table(photo_id),
        )
        print('ImageReceiver.send_msg_clear_photo: photo_id=[{}]'.format(photo_id))
        pass

    def _send_start_receive_photo(self, photo_id: int):
        """
        发送806消息开始接收图像数据
        
        在收到804图像信息消息后调用，通知无人机开始发送数据包。
        使用特殊索引255表示开始传输所有数据包。
        
        Args:
            photo_id: 要接收的照片ID
        """
        self.airplane.send_command_without_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_TOTAL_REQUEST_XINGUANGFEI,
            param1=photo_id,
            param2=255,  # 255是特殊值，表示开始接收所有数据包
        )
        print('ImageReceiver._send_start_receive_photo: photo_id=[{}]'.format(photo_id))
        pass

    def _send_msg_request_missing_packet(self, photo_id: int, packet_index: int):
        """
        发送806消息请求重传丢失的数据包
        
        当检测到丢包时调用，请求无人机重新发送指定索引的数据包。
        
        Args:
            photo_id: 照片ID
            packet_index: 缺失的数据包索引（0-254，255保留为开始传输命令）
        """
        self.airplane.send_command_without_retry(
            mavlink2.MAVLINK_MSG_ID_PHOTO_TOTAL_REQUEST_XINGUANGFEI,
            param1=photo_id,
            param2=packet_index,
        )
        pass

    def on_image_info(self, message: mavlink2.MAVLink_photo_total_information_addr_xinguangfei_message):
        """
        处理804图像信息消息
        
        由AirplaneOwl02收到804消息时调用。
        获取图像的总包数信息，并启动数据传输。
        
        Args:
            message: MAVLink 804消息对象，包含:
                     - photo_id: 照片ID
                     - total_num: 数据包总数
        
        处理流程:
            1. 解析消息获取photo_id和总包数
            2. 创建或更新image_table中的ImageInfo
            3. 发送806消息开始接收数据
        """
        photo_id = message.photo_id
        total_packets = message.total_num
        print('ImageReceiver.on_image_info: photo_id=[{}], total_packets=[{}]'.format(photo_id, total_packets))
        
        # 如果是新图像，创建ImageInfo；否则更新总包数
        if photo_id not in self.image_table:
            self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=total_packets)
        else:
            self.image_table[photo_id].total_packets = total_packets
            pass
        
        # 发送806消息（index=255）开始接收数据包
        self._send_start_receive_photo(photo_id=photo_id)
        pass

    def on_image_packet(self, message: mavlink2.MAVLink_photo_transmission_xinguangfei_message):
        """
        处理805图像数据包消息
        
        由AirplaneOwl02收到805消息时调用。
        缓存数据包，检测丢包，并在所有包接收完成后组装图像。
        
        Args:
            message: MAVLink 805消息对象，包含:
                     - photo_id: 照片ID
                     - index: 数据包索引（0开始）
                     - data: 64字节图像数据
                     - checksum: 校验和
        
        处理流程:
            1. 验证photo_id是否有效
            2. 缓存数据包到packet_cache
            3. 检测乱序/丢包，必要时请求重传
            4. 检查是否所有包都已收到
            5. 如果完成则组装图像
        """
        photo_id = message.photo_id
        packet_index = message.index
        print('ImageReceiver.on_image_packet: photo_id=[{}], packet_index=[{}]'.format(photo_id, packet_index))
        packet_data = bytes(message.data)
        packet_checksum = message.checksum
        
        # 验证photo_id是否在跟踪列表中
        if photo_id not in self.image_table:
            # 未知的photo_id，发送808清除无人机端数据
            self.send_msg_clear_photo(photo_id=photo_id)
            return
        
        image_info = self.image_table[photo_id]
        # 缓存数据包：键为索引，值为(校验和, 数据)元组
        image_info.packet_cache[packet_index] = (packet_checksum, packet_data)
        # 更新最后收包时间
        image_info.last_packet_time = time.time()

        # 如果这个包之前请求过重传，现在收到了，从requested集合中移除
        # 这样如果再次丢失可以重新请求
        if packet_index in image_info.requested_packets:
            image_info.requested_packets.discard(packet_index)

        # ========== 乱序检测和丢包处理 ==========
        # 期望收到的下一个包索引
        expected_index = image_info.max_received_index + 1
        
        # 如果收到的包索引比期望值大很多（超过阈值），认为中间的包丢失了
        if packet_index > expected_index + self.OUT_OF_ORDER_THRESHOLD:
            # 遍历期望索引到当前索引之间的所有索引
            for missing_idx in range(expected_index, packet_index):
                # 如果该索引的包既不在缓存中，也没有正在请求重传
                if missing_idx not in image_info.packet_cache and missing_idx not in image_info.requested_packets:
                    # 标记为已请求，避免重复请求
                    image_info.requested_packets.add(missing_idx)
                    # 发送806请求重传
                    self._send_msg_request_missing_packet(photo_id, missing_idx)

        # 更新已收到的最大包索引
        if packet_index > image_info.max_received_index:
            image_info.max_received_index = packet_index

        # 重置超时定时器，防止误判为传输停滞
        self._reset_timeout_timer(photo_id)

        # 检查是否所有数据包都已接收完成
        if image_info.total_packets > 0 and len(image_info.packet_cache) == image_info.total_packets:
            # 所有包已收到，执行图像组装
            self._complete_image(photo_id)
        pass

    def _reset_timeout_timer(self, photo_id: int):
        """
        重置数据包超时定时器
        
        每次收到新数据包时调用，重置定时器。
        如果在PACKET_TIMEOUT时间内没有收到新包，触发超时处理。
        
        Args:
            photo_id: 当前正在接收的照片ID
        """
        # 取消已存在的定时器
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        
        # 创建新的定时器，超时后调用_on_timeout
        self._timeout_timer = threading.Timer(self.PACKET_TIMEOUT, self._on_timeout, args=[photo_id])
        self._timeout_timer.daemon = True  # 设为守护线程，主线程退出时自动结束
        self._timeout_timer.start()

    def _on_timeout(self, photo_id: int):
        """
        数据包超时处理
        
        当一段时间内没有收到新数据包时被调用。
        检查是否有缺失的包，如有则请求重传；
        如果总超时则强制完成传输。
        
        Args:
            photo_id: 超时的照片ID
        
        处理逻辑:
            1. 检查图像是否已完成
            2. 检查是否总超时，是则强制完成
            3. 遍历所有应收到的包，找出缺失的
            4. 请求重传所有缺失的包
            5. 重置定时器等待重传结果
        """
        # 验证photo_id有效性
        if photo_id not in self.image_table:
            return
        image_info = self.image_table[photo_id]

        # 如果图像数据已组装完成，无需处理
        if image_info.image_data:
            return

        # 如果还未收到804消息（不知道总包数），无法判断缺失
        if image_info.total_packets <= 0:
            return

        # ========== 总超时检查 ==========
        elapsed = time.time() - image_info.start_time
        if elapsed > self.TOTAL_TIMEOUT:
            # 超过总超时时间，强制完成传输
            # 注意：此时图像可能不完整
            self._complete_image(photo_id)
            return

        # ========== 请求重传缺失的包 ==========
        # 清除之前的请求标记，允许重新请求之前请求过但仍未收到的包
        image_info.requested_packets.clear()

        # 遍历所有应有的包索引，找出缺失的
        missing_count = 0
        for i in range(image_info.total_packets):
            if i not in image_info.packet_cache:
                # 标记为已请求
                image_info.requested_packets.add(i)
                # 发送重传请求
                self._send_msg_request_missing_packet(photo_id, i)
                missing_count += 1

        # 根据缺失情况决定后续操作
        if missing_count > 0:
            # 还有缺失的包，重新设置定时器等待重传
            self._reset_timeout_timer(photo_id)
        elif len(image_info.packet_cache) == image_info.total_packets:
            # 所有包都已收到，执行图像组装
            self._complete_image(photo_id)

    def _complete_image(self, photo_id: int):
        """
        完成图像接收并组装数据
        
        当所有数据包都已接收或总超时时调用。
        按索引顺序合并所有数据包，形成完整的JPEG图像。
        
        Args:
            photo_id: 要完成的照片ID
        
        处理流程:
            1. 取消超时定时器
            2. 按顺序合并所有数据包
            3. 调用完成回调通知上层
            4. 发送808清除无人机端数据
        """
        # 验证photo_id有效性
        if photo_id not in self.image_table:
            return
        image_info = self.image_table[photo_id]

        # 取消超时定时器，避免重复处理
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        # ========== 合并所有数据包 ==========
        image_data = bytearray()
        for i in range(image_info.total_packets):
            if i in image_info.packet_cache:
                # 从缓存中取出数据（忽略校验和，仅取数据部分）
                checksum, data = image_info.packet_cache[i]
                image_data.extend(data)
            else:
                # 仍有缺失的包，无法组装完整图像
                # 正常情况不应该执行到这里
                return
        
        # 保存组装完成的图像数据
        image_info.image_data = bytes(image_data)

        # 调用完成回调通知上层应用
        if self._image_complete_callback is not None:
            self._image_complete_callback(photo_id, image_info.image_data)

        # 发送808消息清除无人机端的图像数据，释放存储空间
        self.send_msg_clear_photo(photo_id=photo_id)

    def set_image_complete_callback(self, callback: Optional[Callable[[int, bytes], None]]):
        """
        设置图像接收完成的回调函数
        
        当图像传输完成并组装成功后，会调用此回调函数。
        
        Args:
            callback: 回调函数，签名为 function(photo_id: int, image_data: bytes)
                      - photo_id: 完成的照片ID
                      - image_data: JPEG格式的图像数据
        
        Example:
            def on_image_complete(photo_id, image_data):
                with open(f'photo_{photo_id}.jpg', 'wb') as f:
                    f.write(image_data)
            
            receiver.set_image_complete_callback(on_image_complete)
        """
        self._image_complete_callback = callback

    def get_image(self, photo_id: int) -> bytes | bool:
        """
        获取指定照片ID的图像数据
        
        在调用capture_image()后，可以使用此方法检查图像接收状态
        或获取已完成的图像数据。
        
        Args:
            photo_id: 要查询的照片ID
        
        Returns:
            bytes: 如果图像已接收完成，返回JPEG格式的图像数据
            True: 如果图像正在接收中（尚未完成）
            False: 如果指定的photo_id不存在（无此照片记录）
        
        Example:
            result = receiver.get_image(photo_id)
            if result is False:
                print("照片不存在")
            elif result is True:
                print("照片正在接收中...")
            else:
                # result 是 bytes 类型
                with open('photo.jpg', 'wb') as f:
                    f.write(result)
        """
        if photo_id in self.image_table:
            image_info = self.image_table[photo_id]
            if image_info.image_data:
                # 图像已完成，返回数据
                return image_info.image_data
            else:
                # 图像正在接收中
                return True
        # 照片ID不存在
        return False

    def capture_image(self, callback: Optional[Callable[[int | None], None]] = None):
        """
        发送拍照命令(286)
        
        向无人机发送拍照指令，无人机拍照后会返回807确认消息。
        实际的photo_id会在收到807消息后通过callback返回。
        
        Args:
            callback: 拍照结果回调函数，签名为 function(photo_id: int|None)
                      - 成功时传入分配的photo_id
                      - 失败或超时时传入None
        
        注意事项:
            - 286拍照操作不是幂等的，命令不能重发（会导致重复拍照）
            - 拍照操作具有时效性，命令会立即发送
            - 从发送286到收到807响应约有2-3秒延迟
            - 如果超时（默认20秒）未收到807，认为请求失败
            - 支持多个拍照请求并行等待，按FIFO顺序匹配807响应
        
        使用流程:
            1. 调用capture_image(callback)发送拍照命令
            2. 等待callback被调用，获取photo_id
            3. 如果photo_id不为None，表示拍照成功
            4. 后续会自动接收图像数据，完成后调用image_complete_callback
        
        Example:
            def on_capture_result(photo_id):
                if photo_id is not None:
                    print(f"拍照成功，photo_id={photo_id}")
                else:
                    print("拍照失败或超时")
            
            receiver.capture_image(on_capture_result)
        """
        # 创建等待请求记录，包含回调和超时时间
        pending_request = PendingCaptureRequest(
            callback=callback,
            expire_time=time.time() + self.CAPTURE_TIMEOUT,
        )

        # 将请求加入FIFO队列，等待807响应匹配
        self._pending_capture_requests.append(pending_request)

        # 启动超时检查定时器（如果尚未运行）
        self._start_capture_timeout_checker()

        # 发送286拍照命令
        # 使用send_command_with_retry但max_retries=0，因为拍照不能重发
        self.airplane.send_command_with_retry(
            mavlink2.MAV_CMD_EXT_DRONE_TAKE_PHOTO,
            param1=0,              # cmd参数，默认为0
            timeout=10.0,          # 命令超时时间
            max_retries=0,         # 不重试，因为拍照命令不是幂等的
            async_mode=False,      # 同步模式
            ack_callback=lambda x: print(f'ImageReceiver.capture_image: take photo command ack received', x)
        )
        
        pending_count = len(self._pending_capture_requests)
        print(f'ImageReceiver.capture_image: sent take photo command, pending_count=[{pending_count}], waiting for ack')
        pass

    def _start_capture_timeout_checker(self):
        """
        启动拍照请求超时检查定时器
        
        如果等待队列非空且定时器未运行，则启动定时器。
        定时器会在最早的请求超时时间触发检查。
        """
        # 如果定时器已在运行，不重复启动
        if self._capture_timeout_timer is not None:
            return
        
        # 如果队列为空，无需启动定时器
        if not self._pending_capture_requests:
            return

        # 计算下一个最早超时的时间点
        # FIFO队列中第一个请求的超时时间最早
        next_expire = self._pending_capture_requests[0].expire_time
        delay = max(0.1, next_expire - time.time())  # 至少0.1秒后检查，避免立即触发

        # 创建并启动定时器
        self._capture_timeout_timer = threading.Timer(delay, self._check_capture_timeouts)
        self._capture_timeout_timer.daemon = True  # 守护线程
        self._capture_timeout_timer.start()

    def _check_capture_timeouts(self):
        """
        检查并处理超时的拍照请求
        
        定时器触发时调用，从队列头部开始检查超时的请求。
        超时的请求会被移除，并调用其回调函数传入None。
        """
        # 清除定时器引用
        self._capture_timeout_timer = None
        current_time = time.time()

        # 从队列头部开始检查，移除所有已超时的请求
        while self._pending_capture_requests:
            request = self._pending_capture_requests[0]
            if request.expire_time <= current_time:
                # 请求已超时，从队列移除
                self._pending_capture_requests.popleft()
                print(f'ImageReceiver._check_capture_timeouts: capture request timed out')
                # 调用回调通知超时（传入None）
                if request.callback is not None:
                    request.callback(None)
            else:
                # 队列头部未超时，由于是FIFO队列，后面的也不会超时
                # 退出循环
                break

        # 如果队列中仍有等待的请求，继续启动定时器
        self._start_capture_timeout_checker()

    def on_take_photo_ack(self, message: mavlink2.MAVLink_take_photo_ack_xinguangfei_message):
        """
        处理807拍照确认消息
        
        由AirplaneOwl02收到807消息时调用。
        按FIFO顺序匹配等待队列中的请求，并调用相应的回调函数。
        
        Args:
            message: MAVLink 807消息对象，包含:
                     - photo_id: 分配的照片ID
                     - result: 拍照结果（0=成功，其他=失败）
        
        处理逻辑:
            1. 如果等待队列非空，取出队首请求（FIFO匹配）
               - result=0: 拍照成功，初始化ImageInfo，回调传入photo_id
               - result!=0: 拍照失败，回调传入None
            2. 如果等待队列为空（意外的807消息）
               - 发送808(id=0)清除远端所有图片数据
               - 避免无人机端存储泄漏
        
        注意:
            807消息与286请求是按顺序一一对应的，使用FIFO队列匹配
        """
        photo_id = message.photo_id
        result = message.result
        print(f'ImageReceiver.on_take_photo_ack: photo_id=[{photo_id}], result=[{result}]')

        # 检查是否有等待中的拍照请求
        if self._pending_capture_requests:
            # 从队列头部取出最早的请求（FIFO顺序匹配）
            pending_request = self._pending_capture_requests.popleft()
            print(f'ImageReceiver.on_take_photo_ack: matched pending request')

            if result == 0:
                # ========== 拍照成功 ==========
                # 初始化image_table条目，准备接收图像数据
                if photo_id not in self.image_table:
                    self.image_table[photo_id] = ImageInfo(photo_id=photo_id, total_packets=0)
                
                # 调用回调函数，传入分配的photo_id
                if pending_request.callback is not None:
                    pending_request.callback(photo_id)
            else:
                # ========== 拍照失败 ==========
                # 调用回调函数，传入None表示失败
                if pending_request.callback is not None:
                    pending_request.callback(None)
        else:
            # ========== 意外的807消息 ==========
            # 没有等待的请求，可能是之前的请求已超时被移除
            # 为避免无人机端存储泄漏，发送808清除所有图片数据
            print('ImageReceiver.on_take_photo_ack: no pending request, clearing remote photo data')
            self.send_msg_clear_photo(photo_id=0)
        pass
