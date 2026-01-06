import dataclasses
import functools
from typing import Optional, Callable

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

    def __init__(self, airplane: 'AirplaneOwl02'):
        self.airplane = airplane
        self.image_table = {}
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
        )
        pass

    def send_msg_request_missing_packet(self, photo_id: int, packet_index: int):
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
            # TODO clean this image 808
            self.send_msg_clear_photo(photo_id=photo_id)
            return
        image_info = self.image_table[photo_id]
        image_info.packet_cache[packet_index] = (packet_checksum, packet_data)
        # TODO check checksum and request missing packets 806

        # check if all packets received
        if len(image_info.packet_cache) == image_info.total_packets:
            # combine image data
            image_data = bytearray()
            for i in range(image_info.total_packets):
                if i in image_info.packet_cache:
                    checksum, data = image_info.packet_cache[i]
                    image_data.extend(data)
                else:
                    # missing packet, should not happen here
                    return
            image_info.image_data = bytes(image_data)
            # send 808 to clear photo data in drone
            self.send_msg_clear_photo(photo_id=photo_id)
            # TODO notify image received , callback
        pass

    def get_image(self, photo_id: int) -> bytes | None:
        """
        get received image data by photo_id
        :param photo_id:
        :return: image data in bytes, or None if not found or not complete
        """
        if photo_id in self.image_table:
            image_info = self.image_table[photo_id]
            if image_info.image_data:
                return image_info.image_data
        return None

    def capture_image(self, callback: Optional[Callable[[int|None], None]] = None) -> int:
        """
        send command 286 to capture image
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
                                callback: Optional[Callable[[int|None], None]] = None) -> None:
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
