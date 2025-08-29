from time import sleep
from typing import Union, Optional
from airplane_manager_owl02 import get_airplane_manager_owl02, AirplaneManagerOwl02
from airplane_owl02 import AirplaneOwl02


class Owl02:
    """
    OWL02无人机控制器，对AirplaneManagerOwl02的封装
    提供与ph0apy.py兼容的API接口
    """

    def __init__(self) -> None:
        """初始化OWL02控制器"""
        self.manager: AirplaneManagerOwl02 = get_airplane_manager_owl02()
        self.manager.init()
        pass

    def sleep(self, time: float) -> None:
        """休眠指定时间 单位：秒"""
        sleep(time)
        pass

    def destroy(self) -> None:
        """销毁管理器，释放资源"""
        self.manager.stop()
        pass

    def add_uav(self, uav_id: str) -> None:
        """添加（注册）无人机
        :param uav_id: 无人机ID，格式必须为 "COMx:id" (如 "COM6:3")，其中id在1-16之间
        """
        # 将uav_id转换为设备ID
        device_id = self._convert_to_device_id(uav_id)
        self.manager.get_airplane(device_id)
        pass

    def _convert_to_device_id(self, uav_id: str) -> int:
        """将无人机ID转换为内部设备ID
        :param uav_id: 无人机ID，格式必须为 "COMx:id"，其中id在1-16之间
        :return: 设备ID (1-16之间的整数)
        """
        try:
            # 检查格式是否为 "COMx:id"
            if not isinstance(uav_id, str) or ':' not in uav_id:
                raise ValueError(f"无人机ID格式错误，必须为 'COMx:id' 格式，当前值: {uav_id}")

            # 解析 "COMx:id" 格式
            port_part, id_part = uav_id.split(':', 1)

            # 验证端口部分格式
            if not port_part.upper().startswith('COM'):
                raise ValueError(f"端口部分必须以COM开头，当前值: {port_part}")

            # 解析ID部分
            device_id = int(id_part)
            if 1 <= device_id <= 16:
                return device_id
            else:
                raise ValueError(f"无人机ID必须在1-16之间，当前值: {device_id}")

        except ValueError as e:
            if "无人机ID必须在1-16之间" in str(e) or "端口部分必须以COM开头" in str(e) or "无人机ID格式错误" in str(e):
                raise e
            else:
                raise ValueError(f"无法解析无人机ID: {uav_id}，必须为 'COMx:id' 格式，其中x为端口号，id为1-16之间的数字")

    def p(self, uav_id: str) -> AirplaneOwl02:
        """获取无人机对象
        :param uav_id: 无人机ID，格式必须为 "COMx:id" (如 "COM6:3")
        :return: 无人机对象
        """
        device_id = self._convert_to_device_id(uav_id)
        return self.manager.get_airplane(device_id)

    def land(self, uav_id: str) -> None:
        """降落
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        """
        self.p(uav_id).land()

    # def emergency(self, uav_id: str) -> None:
    #     self.p(uav_id).stop()

    def takeoff(self, uav_id: str, height: int) -> None:
        """起飞到指定高度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param height: 目标高度，单位：厘米
        """
        drone = self.p(uav_id)
        drone.takeoff(height / 100.0)  # 转换厘米到米

    def up(self, uav_id: str, distance: int) -> None:
        """上升指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 上升距离，单位：厘米
        """
        self.p(uav_id).up(distance)

    def down(self, uav_id: str, distance: int) -> None:
        """下降指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 下降距离，单位：厘米
        """
        self.p(uav_id).down(distance)

    def forward(self, uav_id: str, distance: int) -> None:
        """前进指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 前进距离，单位：厘米
        """
        self.p(uav_id).forward(distance)

    def back(self, uav_id: str, distance: int) -> None:
        """后退指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 后退距离，单位：厘米
        """
        self.p(uav_id).back(distance)

    def left(self, uav_id: str, distance: int) -> None:
        """左移指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 左移距离，单位：厘米
        """
        self.p(uav_id).left(distance)

    def right(self, uav_id: str, distance: int) -> None:
        """右移指定距离
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param distance: 右移距离，单位：厘米
        """
        self.p(uav_id).right(distance)

    def goto(self, uav_id: str, x: int, y: int, h: int) -> None:
        """移动到指定坐标位置
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param x: X坐标，单位：厘米
        :param y: Y坐标，单位：厘米
        :param h: 高度，单位：厘米
        """
        self.p(uav_id).goto(x, y, h)

    def flip(self, uav_id: str, direction: Optional[str] = None) -> None:
        """控制无人机翻滚
        支持两种调用方式：
        1. flip(uav_id, direction) - 标准方式
        :param uav_id: 无人机ID（格式必须为 "COMx:id"）或者翻滚方向
        :param direction: 翻滚方向（f前 b后 l左 r右），仅在第一种调用方式时使用
        """
        drone = self.p(uav_id)
        if direction == 'f':
            drone.flip_forward()
        elif direction == 'b':
            drone.flip_back()
        elif direction == 'r':
            drone.flip_right()
        elif direction == 'l':
            drone.flip_left()
        pass

    def rotate(self, uav_id: str, degree: int) -> None:
        """旋转指定角度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param degree: 旋转角度，正值表示顺时针
        """
        self.p(uav_id).rotate(degree)

    def cw(self, uav_id: str, degree: int) -> None:
        """顺时针旋转指定角度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param degree: 旋转角度
        """
        self.p(uav_id).cw(degree)

    def ccw(self, uav_id: str, degree: int) -> None:
        """逆时针旋转指定角度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param degree: 旋转角度
        """
        self.p(uav_id).ccw(degree)

    def speed(self, uav_id: str, speed_value: int) -> None:
        """设置飞行速度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param speed_value: 飞行速度值
        """
        self.p(uav_id).speed(speed_value)

    def high(self, uav_id: str, height: int) -> None:
        """移动到指定高度
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param height: 目标高度，单位：厘米
        """
        self.p(uav_id).high(height)

    def led(self, uav_id: str, r: int, g: int, b: int) -> None:
        """设置无人机LED色彩
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param r: 红色分量 (0-255)
        :param g: 绿色分量 (0-255)
        :param b: 蓝色分量 (0-255)
        """
        self.p(uav_id).led(r, g, b)

    def bln(self, uav_id: str, r: int, g: int, b: int) -> None:
        """设置无人机LED呼吸灯色彩
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param r: 红色分量 (0-255)
        :param g: 绿色分量 (0-255)
        :param b: 蓝色分量 (0-255)
        """
        self.p(uav_id).bln(r, g, b)

    def rainbow(self, uav_id: str, r: int, g: int, b: int) -> None:
        """设置无人机LED彩虹色彩
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param r: 红色分量 (0-255)
        :param g: 绿色分量 (0-255)
        :param b: 蓝色分量 (0-255)
        """
        self.p(uav_id).rainbow(r, g, b)

    def mode(self, uav_id: str, flight_mode: int) -> None:
        """设置无人机飞行模式
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param flight_mode: 飞行模式 (1常规 2巡线 3跟随 4单机编队)
        """
        self.p(uav_id).airplane_mode(flight_mode)
        pass

    def color_detect(self, uav_id: str, L_L: int, L_H: int, A_L: int, A_H: int, B_L: int, B_H: int) -> None:
        """颜色检测，检测指定颜色
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param L_L: L通道下限
        :param L_H: L通道上限
        :param A_L: A通道下限
        :param A_H: A通道上限
        :param B_L: B通道下限
        :param B_H: B通道上限
        
        注意：此功能在OWL02后端中可能需要额外实现
        """
        # TODO: 实现颜色检测功能
        pass

    # def color_detect_label(self, uav_id: str, label: str) -> None:
    #     self.p(uav_id).color_detect_label(label)

    def vision_mode(self, uav_id: str, vision_mode: int) -> None:
        """设置视觉工作模式
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        :param vision_mode: 视觉模式 (1点检测 2线检测 3标签检测 4二维码扫描 5条形码扫描)
        
        注意：此功能在OWL02后端中可能需要额外实现
        """
        # TODO: 实现视觉模式设置
        pass

    def stop(self, uav_id: str) -> None:
        """紧急停桨
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        """
        self.p(uav_id).stop()

    def hover(self, uav_id: str) -> None:
        """悬停
        :param uav_id: 无人机ID，格式必须为 "COMx:id"
        """
        self.p(uav_id).hover()

    pass
