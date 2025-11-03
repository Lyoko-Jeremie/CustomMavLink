"""
无人机控制GUI程序 - 简单调试界面
使用tkinter创建图形界面，每个按钮对应一个控制指令
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import logging
from datetime import datetime
from typing import Optional
import time
from owl2.airplane_manager_owl02 import create_manager_with_serial, AirplaneOwl02
try:
    # pyserial provides a cross-platform way to list serial ports
    from serial.tools import list_ports
except Exception:
    list_ports = None

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 新增：命令任务与管理队列
from dataclasses import dataclass, field
from typing import Any, Callable, Tuple, Dict

@dataclass
class CommandTask:
    """表示要发送给单台无人机的命令任务。"""
    drone_id: int
    command: str
    args: Tuple[Any, ...] = field(default_factory=tuple)
    kwargs: Dict[str, Any] = field(default_factory=dict)
    retries: int = 1
    on_done: Optional[Callable[[bool, Optional[Exception]], None]] = None

class ManagerCommandQueue:
    """并发命令队列：使用线程池并发处理任务，但对实际串口写操作使用锁序列化，避免冲突。

    这样可以并发发送/等待响应（IO并发），但保证写入串口的操作被保护。
    """
    def __init__(self, manager, max_workers: int = 8):
        self.manager = manager
        # 用于保护对串口的写操作（如果 manager/airplane 的方法执行串口写）
        self._write_lock = threading.Lock()
        # 线程池用于并发处理任务（发送/接收）
        import concurrent.futures
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
        self._shutdown = False

    def enqueue(self, task: CommandTask):
        if self._shutdown:
            return
        # 提交任务到线程池并立即返回（非阻塞）
        self._executor.submit(self._process_task, task)

    def _process_task(self, task: CommandTask):
        """在独立线程中处理单个任务。"""
        attempt = 0
        last_exc = None
        while attempt <= (task.retries or 0):
            try:
                attempt += 1
                airplane = None
                if hasattr(self.manager, 'get_airplane'):
                    try:
                        airplane = self.manager.get_airplane(task.drone_id)
                    except Exception:
                        airplane = None

                # 在对串口进行写操作时加锁，防止并发写导致数据混淆
                with self._write_lock:
                    if airplane is not None and hasattr(airplane, task.command):
                        getattr(airplane, task.command)(*task.args, **task.kwargs)
                    elif hasattr(self.manager, 'send_command'):
                        # manager 层可能提供统一发送接口
                        self.manager.send_command(task.drone_id, task.command, *task.args, **task.kwargs)
                    else:
                        raise AttributeError(f"无人机对象或管理器不支持命令 {task.command}")

                # 如果调用成功就调用回调并退出
                if task.on_done:
                    try:
                        task.on_done(True, None)
                    except Exception:
                        pass
                return
            except Exception as e:
                last_exc = e
                time.sleep(0.05)
                if attempt > (task.retries or 0):
                    if task.on_done:
                        try:
                            task.on_done(False, e)
                        except Exception:
                            pass
        # 结束

    def stop(self, wait=True):
        self._shutdown = True
        try:
            self._executor.shutdown(wait=wait)
        except Exception:
            pass


class DroneControlGUI:
    """无人机控制GUI类"""

    def __init__(self, root):
        self.root = root
        self.root.title("无人机控制调试界面")
        self.root.geometry("1400x950")  # 增加窗口宽度以适应三栏布局

        self.manager = None
        self.drone: Optional[AirplaneOwl02] = None
        self.drone_id = 2

        # 新增：多选复选框状态字典和命令队列引用（在 manager 初始化后创建队列）
        self.id_check_vars = {}  # key: id -> tk.IntVar
        self.cmd_queue: Optional[ManagerCommandQueue] = None

        self.setup_ui()

    def setup_ui(self):
        """设置界面布局"""
        # 标题
        title_label = tk.Label(
            self.root,
            text="无人机控制调试面板",
            font=("Arial", 16, "bold"),
            pady=10
        )
        title_label.pack()

        # 创建主容器 - 三栏布局
        main_container = tk.Frame(self.root)
        main_container.pack(fill="both", expand=True, padx=10, pady=5)

        # 左侧面板 - 基本控制
        left_panel = tk.Frame(main_container, width=400)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 中间面板 - 高级功能
        middle_panel = tk.Frame(main_container, width=400)
        middle_panel.pack(side="left", fill="both", expand=True, padx=5)

        # 右侧面板 - 日志输出
        right_panel = tk.Frame(main_container, width=400)
        right_panel.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # ==================== 左侧内容 - 基本控制 ====================
        # 初始化区域
        init_frame = ttk.LabelFrame(left_panel, text="初始化", padding=10)
        init_frame.pack(fill="x", pady=5)

        # COM口配置
        com_frame = tk.Frame(init_frame)
        com_frame.pack(fill="x", pady=5)

        tk.Label(com_frame, text="COM口:").pack(side="left", padx=5)
        # 使用下拉框列出当前可用的COM口
        self.com_port_combo = ttk.Combobox(com_frame, width=15, state="readonly")
        self._populate_com_ports()
        self.com_port_combo.pack(side="left", padx=5)

        # 刷新端口列表按钮
        refresh_btn = tk.Button(com_frame, text="刷新", command=self._populate_com_ports, width=6)
        refresh_btn.pack(side="left", padx=5)

        tk.Label(com_frame, text="波特率:").pack(side="left", padx=5)
        self.baudrate_entry = tk.Entry(com_frame, width=10)
        self.baudrate_entry.insert(0, "921600")
        self.baudrate_entry.pack(side="left", padx=5)

        # 无人机ID输入
        id_frame = tk.Frame(init_frame)
        id_frame.pack(fill="x", pady=5)
        tk.Label(id_frame, text="无人机ID:").pack(side="left", padx=5)
        # 使用下拉框选择无人机ID（默认0-16）。如果初始化了manager，会尝试使用管理器提供的列表刷新此下拉框。
        self.id_combo = ttk.Combobox(id_frame, width=10, state="readonly")
        # 预填默认选项（0..16）并设置默认值为2
        # 移除无人机 16，默认只显示 0..15
        self.id_combo['values'] = [str(i) for i in range(0, 16)]
        self.id_combo.set(str(self.drone_id))
        self.id_combo.pack(side="left", padx=5)

        # 新增：在初始化区域下面显示 0..16 的多选复选框，支持全选/反选
        self._create_id_checkpanel(init_frame)

        # 初始化按钮
        # 初始化按钮
        self.btn_init = tk.Button(
            init_frame,
            text="初始化管理器",
            command=self.init_manager,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"),
            width=20,
            height=2
        )
        self.btn_init.pack(pady=5)

        btn_get_drone = tk.Button(
            init_frame,
            text="获取无人机对象",
            command=self.get_drone,
            bg="#2196F3",
            fg="white",
            font=("Arial", 10, "bold"),
            width=20,
            height=2
        )
        btn_get_drone.pack(pady=5)

        # 断开连接按钮，初始禁用
        self.btn_disconnect = tk.Button(
            init_frame,
            text="断开连接并重置",
            command=self.disconnect_and_reset,
            bg="#F44336",
            fg="white",
            font=("Arial", 10, "bold"),
            width=20,
            height=2,
            state='disabled'
        )
        self.btn_disconnect.pack(pady=5)

        # 心跳包控制区域
        heartbeat_frame = tk.Frame(init_frame)
        heartbeat_frame.pack(fill="x", pady=5)

        tk.Label(heartbeat_frame, text="心跳包:").pack(side="left", padx=5)

        self.heartbeat_var = tk.BooleanVar(value=True)  # 默认启用
        self.heartbeat_checkbox = tk.Checkbutton(
            heartbeat_frame,
            text="启用心跳包发送",
            variable=self.heartbeat_var,
            command=self.toggle_heartbeat,
            font=("Arial", 10)
        )
        self.heartbeat_checkbox.pack(side="left", padx=5)

        # 心跳状态指示器
        self.heartbeat_indicator = tk.Label(
            heartbeat_frame,
            text="●",
            fg="#4CAF50",
            font=("Arial", 16)
        )
        self.heartbeat_indicator.pack(side="left", padx=5)

        # 基本控制区域
        basic_frame = ttk.LabelFrame(left_panel, text="基本控制", padding=10)
        basic_frame.pack(fill="x", pady=5)

        # 第一行：解锁和上锁
        row1 = tk.Frame(basic_frame)
        row1.pack(fill="x", pady=5)

        tk.Button(
            row1, text="解锁 (Arm)", command=self.arm,
            bg="#FF9800", fg="white", font=("Arial", 10, "bold"),
            width=15, height=2
        ).pack(side="left", padx=5, expand=True)

        tk.Button(
            row1, text="上锁 (Disarm)", command=self.disarm,
            bg="#9E9E9E", fg="white", font=("Arial", 10, "bold"),
            width=15, height=2
        ).pack(side="left", padx=5, expand=True)

        # 第二行：起飞和降落
        row2 = tk.Frame(basic_frame)
        row2.pack(fill="x", pady=5)

        # 起飞高度输入
        tk.Label(row2, text="高度(cm):").pack(side="left", padx=5)
        self.takeoff_height = tk.Entry(row2, width=8)
        self.takeoff_height.insert(0, "150")
        self.takeoff_height.pack(side="left", padx=5)

        tk.Button(
            row2, text="起飞 (Takeoff)", command=self.takeoff,
            bg="#8BC34A", fg="white", font=("Arial", 10, "bold"),
            width=12, height=2
        ).pack(side="left", padx=5, expand=True)

        tk.Button(
            row2, text="降落 (Land)", command=self.land,
            bg="#FF5722", fg="white", font=("Arial", 10, "bold"),
            width=12, height=2
        ).pack(side="left", padx=5, expand=True)

        # 移动控制区域
        move_frame = ttk.LabelFrame(left_panel, text="移动控制", padding=10)
        move_frame.pack(fill="x", pady=5)

        # 移动距离输入
        distance_frame = tk.Frame(move_frame)
        distance_frame.pack(fill="x", pady=5)
        tk.Label(distance_frame, text="移动距离(cm):").pack(side="left", padx=5)
        self.move_distance = tk.Entry(distance_frame, width=10)
        self.move_distance.insert(0, "100")
        self.move_distance.pack(side="left", padx=5)

        # 方向控制按钮
        direction_grid = tk.Frame(move_frame)
        direction_grid.pack(pady=5)

        # 上升
        tk.Button(
            direction_grid, text="↑ 上升 (Up)", command=self.up,
            bg="#03A9F4", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=0, column=1, padx=5, pady=5)

        # 前进
        tk.Button(
            direction_grid, text="↑ 前进 (Forward)", command=self.forward,
            bg="#009688", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=1, column=1, padx=5, pady=5)

        # 左右
        tk.Button(
            direction_grid, text="← 左移 (Left)", command=self.left,
            bg="#009688", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=2, column=0, padx=5, pady=5)

        tk.Button(
            direction_grid, text="→ 右移 (Right)", command=self.right,
            bg="#009688", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=2, column=2, padx=5, pady=5)

        # 后退
        tk.Button(
            direction_grid, text="↓ 后退 (Back)", command=self.back,
            bg="#009688", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=3, column=1, padx=5, pady=5)

        # 下降
        tk.Button(
            direction_grid, text="↓ 下降 (Down)", command=self.down,
            bg="#03A9F4", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).grid(row=4, column=1, padx=5, pady=5)

        # ==================== 中间内容 - 高级功能 ====================
        # Goto控制区域
        goto_frame = ttk.LabelFrame(middle_panel, text="定点飞行", padding=10)
        goto_frame.pack(fill="x", pady=5)

        coords_frame = tk.Frame(goto_frame)
        coords_frame.pack(fill="x", pady=5)

        tk.Label(coords_frame, text="X(cm):").pack(side="left", padx=5)
        self.goto_x = tk.Entry(coords_frame, width=8)
        self.goto_x.insert(0, "100")
        self.goto_x.pack(side="left", padx=5)

        tk.Label(coords_frame, text="Y(cm):").pack(side="left", padx=5)
        self.goto_y = tk.Entry(coords_frame, width=8)
        self.goto_y.insert(0, "100")
        self.goto_y.pack(side="left", padx=5)

        tk.Label(coords_frame, text="Z(cm):").pack(side="left", padx=5)
        self.goto_z = tk.Entry(coords_frame, width=8)
        self.goto_z.insert(0, "150")
        self.goto_z.pack(side="left", padx=5)

        tk.Button(
            goto_frame, text="飞往目标点 (Goto)", command=self.goto,
            bg="#673AB7", fg="white", font=("Arial", 10, "bold"),
            width=20, height=2
        ).pack(pady=5)

        # 灯光控制区域
        light_frame = ttk.LabelFrame(middle_panel, text="灯光控制", padding=10)
        light_frame.pack(fill="x", pady=5)

        # RGB输入
        rgb_input_frame = tk.Frame(light_frame)
        rgb_input_frame.pack(fill="x", pady=5)

        tk.Label(rgb_input_frame, text="R:").pack(side="left", padx=2)
        self.light_r = tk.Entry(rgb_input_frame, width=6)
        self.light_r.insert(0, "255")
        self.light_r.pack(side="left", padx=2)

        tk.Label(rgb_input_frame, text="G:").pack(side="left", padx=2)
        self.light_g = tk.Entry(rgb_input_frame, width=6)
        self.light_g.insert(0, "0")
        self.light_g.pack(side="left", padx=2)

        tk.Label(rgb_input_frame, text="B:").pack(side="left", padx=2)
        self.light_b = tk.Entry(rgb_input_frame, width=6)
        self.light_b.insert(0, "0")
        self.light_b.pack(side="left", padx=2)

        # 颜色预览
        self.color_preview = tk.Label(
            rgb_input_frame, text="  ", bg="#FF0000", width=3, relief="solid", borderwidth=1
        )
        self.color_preview.pack(side="left", padx=5)

        # 绑定颜色输入框的事件，实时更新预览
        self.light_r.bind("<KeyRelease>", self.update_color_preview)
        self.light_g.bind("<KeyRelease>", self.update_color_preview)
        self.light_b.bind("<KeyRelease>", self.update_color_preview)

        # 灯光模式按钮
        light_mode_frame = tk.Frame(light_frame)
        light_mode_frame.pack(fill="x", pady=5)

        tk.Button(
            light_mode_frame, text="常亮 (LED)", command=self.set_led,
            bg="#FFC107", fg="black", font=("Arial", 9, "bold"),
            width=12, height=2
        ).pack(side="left", padx=3, expand=True)

        tk.Button(
            light_mode_frame, text="呼吸灯 (Breathe)", command=self.set_breathe,
            bg="#00BCD4", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).pack(side="left", padx=3, expand=True)

        tk.Button(
            light_mode_frame, text="彩虹灯 (Rainbow)", command=self.set_rainbow,
            bg="#E91E63", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).pack(side="left", padx=3, expand=True)

        # 预设颜色按钮
        preset_frame = tk.Frame(light_frame)
        preset_frame.pack(fill="x", pady=5)

        preset_colors = [
            ("红", "#FF0000", 255, 0, 0),
            ("绿", "#00FF00", 0, 255, 0),
            ("蓝", "#0000FF", 0, 0, 255),
            ("黄", "#FFFF00", 255, 255, 0),
            ("紫", "#FF00FF", 255, 0, 255),
            ("青", "#00FFFF", 0, 255, 255),
            ("白", "#FFFFFF", 255, 255, 255),
            ("关", "#000000", 0, 0, 0),
        ]

        for name, color, r, g, b in preset_colors:
            btn = tk.Button(
                preset_frame, text=name, bg=color,
                fg="white" if sum([r, g, b]) < 400 else "black",
                font=("Arial", 8, "bold"),
                width=4, height=1,
                command=lambda r=r, g=g, b=b: self.set_preset_color(r, g, b)
            )
            btn.pack(side="left", padx=2)

        # 飞行模式设置区域
        mode_frame = ttk.LabelFrame(middle_panel, text="飞行模式设置", padding=10)
        mode_frame.pack(fill="x", pady=5)

        mode_desc_label = tk.Label(
            mode_frame,
            text="飞行模式：1-常规模式  2-巡线模式  3-跟随模式",
            font=("Arial", 8),
            fg="#666666"
        )
        mode_desc_label.pack(pady=2)

        mode_buttons_frame = tk.Frame(mode_frame)
        mode_buttons_frame.pack(fill="x", pady=5)

        tk.Button(
            mode_buttons_frame, text="常规模式", command=lambda: self.set_flight_mode(1),
            bg="#4CAF50", fg="white", font=("Arial", 9, "bold"),
            width=10, height=2
        ).pack(side="left", padx=5, expand=True)

        tk.Button(
            mode_buttons_frame, text="巡线模式", command=lambda: self.set_flight_mode(2),
            bg="#FF9800", fg="white", font=("Arial", 9, "bold"),
            width=10, height=2
        ).pack(side="left", padx=5, expand=True)

        tk.Button(
            mode_buttons_frame, text="跟随模式", command=lambda: self.set_flight_mode(3),
            bg="#2196F3", fg="white", font=("Arial", 9, "bold"),
            width=10, height=2
        ).pack(side="left", padx=5, expand=True)

        # 色块检测设置区域
        detect_frame = ttk.LabelFrame(middle_panel, text="色块检测设置 (LAB颜色空间)", padding=10)
        detect_frame.pack(fill="x", pady=5)

        detect_desc_label = tk.Label(
            detect_frame,
            text="设置LAB颜色空间的检测范围",
            font=("Arial", 8),
            fg="#666666"
        )
        detect_desc_label.pack(pady=2)

        # L通道
        l_frame = tk.Frame(detect_frame)
        l_frame.pack(fill="x", pady=2)
        tk.Label(l_frame, text="L通道:", width=8).pack(side="left", padx=2)
        tk.Label(l_frame, text="最小:").pack(side="left", padx=2)
        self.detect_l_min = tk.Entry(l_frame, width=6)
        self.detect_l_min.insert(0, "0")
        self.detect_l_min.pack(side="left", padx=2)
        tk.Label(l_frame, text="最大:").pack(side="left", padx=2)
        self.detect_l_max = tk.Entry(l_frame, width=6)
        self.detect_l_max.insert(0, "100")
        self.detect_l_max.pack(side="left", padx=2)

        # A通道
        a_frame = tk.Frame(detect_frame)
        a_frame.pack(fill="x", pady=2)
        tk.Label(a_frame, text="A通道:", width=8).pack(side="left", padx=2)
        tk.Label(a_frame, text="最小:").pack(side="left", padx=2)
        self.detect_a_min = tk.Entry(a_frame, width=6)
        self.detect_a_min.insert(0, "-128")
        self.detect_a_min.pack(side="left", padx=2)
        tk.Label(a_frame, text="最大:").pack(side="left", padx=2)
        self.detect_a_max = tk.Entry(a_frame, width=6)
        self.detect_a_max.insert(0, "127")
        self.detect_a_max.pack(side="left", padx=2)

        # B通道
        b_frame = tk.Frame(detect_frame)
        b_frame.pack(fill="x", pady=2)
        tk.Label(b_frame, text="B通道:", width=8).pack(side="left", padx=2)
        tk.Label(b_frame, text="最小:").pack(side="left", padx=2)
        self.detect_b_min = tk.Entry(b_frame, width=6)
        self.detect_b_min.insert(0, "-128")
        self.detect_b_min.pack(side="left", padx=2)
        tk.Label(b_frame, text="最大:").pack(side="left", padx=2)
        self.detect_b_max = tk.Entry(b_frame, width=6)
        self.detect_b_max.insert(0, "127")
        self.detect_b_max.pack(side="left", padx=2)

        # 预设颜色检测按钮
        detect_preset_frame = tk.Frame(detect_frame)
        detect_preset_frame.pack(fill="x", pady=5)

        detect_presets = [
            ("红色", 0, 100, 20, 127, -128, 127),
            ("绿色", 0, 100, -128, -20, -128, 127),
            ("蓝色", 0, 100, -128, 127, -128, -20),
        ]

        for name, l_min, l_max, a_min, a_max, b_min, b_max in detect_presets:
            btn = tk.Button(
                detect_preset_frame, text=name,
                font=("Arial", 8),
                width=8, height=1,
                command=lambda lmin=l_min, lmax=l_max, amin=a_min, amax=a_max, bmin=b_min, bmax=b_max:
                    self.set_detect_preset(lmin, lmax, amin, amax, bmin, bmax)
            )
            btn.pack(side="left", padx=3)

        tk.Button(
            detect_frame, text="应用色块检测设置", command=self.apply_color_detect,
            bg="#9C27B0", fg="white", font=("Arial", 9, "bold"),
            width=20, height=2
        ).pack(pady=5)

        # 旋转控制面板
        rotate_frame = ttk.LabelFrame(middle_panel, text="旋转控制", padding=10)
        rotate_frame.pack(fill="x", pady=5)

        rotate_input_frame = tk.Frame(rotate_frame)
        rotate_input_frame.pack(fill="x", pady=5)

        tk.Label(rotate_input_frame, text="角度(度):").pack(side="left", padx=5)
        self.rotate_angle = tk.Entry(rotate_input_frame, width=8)
        # 默认角度为90度
        self.rotate_angle.insert(0, "90")
        self.rotate_angle.pack(side="left", padx=5)

        rotate_btn_frame = tk.Frame(rotate_frame)
        rotate_btn_frame.pack(pady=5)

        tk.Button(
            rotate_btn_frame, text="顺时针 (CW)", command=self.rotate_cw,
            bg="#4CAF50", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).pack(side="left", padx=5, expand=True)

        tk.Button(
            rotate_btn_frame, text="逆时针 (CCW)", command=self.rotate_ccw,
            bg="#2196F3", fg="white", font=("Arial", 9, "bold"),
            width=12, height=2
        ).pack(side="left", padx=5, expand=True)

        # ==================== 右侧内容 - 日志输出 ====================

        # 翻滚控制面板（移到右侧，放在日志输出上方）
        flip_frame = ttk.LabelFrame(right_panel, text="翻滚控制", padding=10)
        flip_frame.pack(fill="x", pady=5)

        flip_btn_frame = tk.Frame(flip_frame)
        flip_btn_frame.pack(fill="x", pady=5)

        # 四个翻滚按钮一排排列
        tk.Button(
            flip_btn_frame, text="前翻 (Flip Forward)", command=self.flip_forward,
            bg="#FF5722", fg="white", font=("Arial", 9, "bold"),
            width=14, height=2
        ).pack(side="left", padx=5, pady=3, expand=True, fill='x')

        tk.Button(
            flip_btn_frame, text="后翻 (Flip Back)", command=self.flip_back,
            bg="#9E9E9E", fg="white", font=("Arial", 9, "bold"),
            width=14, height=2
        ).pack(side="left", padx=5, pady=3, expand=True, fill='x')

        tk.Button(
            flip_btn_frame, text="左翻 (Flip Left)", command=self.flip_left,
            bg="#03A9F4", fg="white", font=("Arial", 9, "bold"),
            width=14, height=2
        ).pack(side="left", padx=5, pady=3, expand=True, fill='x')

        tk.Button(
            flip_btn_frame, text="右翻 (Flip Right)", command=self.flip_right,
            bg="#009688", fg="white", font=("Arial", 9, "bold"),
            width=14, height=2
        ).pack(side="left", padx=5, pady=3, expand=True, fill='x')

        # 日志输出区域
        log_frame = ttk.LabelFrame(right_panel, text="日志输出", padding=10)
        log_frame.pack(fill="both", expand=True)

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=10,
            wrap=tk.WORD,
            font=("Consolas", 9)
        )
        self.log_text.pack(fill="both", expand=True)

        # 清空日志按钮
        tk.Button(
            log_frame, text="清空日志", command=self.clear_log,
            bg="#607D8B", fg="white", width=15
        ).pack(pady=5)

        # 底部状态栏（包含状态文本和当前控制的无人机ID）
        status_frame = tk.Frame(self.root, bg="#F0F0F0")
        status_frame.pack(fill="x", side="bottom")

        self.status_label = tk.Label(
            status_frame,
            text="状态: 未初始化",
            bg="#F0F0F0",
            anchor="w",
            padx=10
        )
        self.status_label.pack(side="left", fill="x", expand=True)

        # 显示当前已获取并正在控制的无人机ID，默认为空（—）
        self.drone_id_label = tk.Label(
            status_frame,
            text="当前无人机: —",
            bg="#F0F0F0",
            anchor="e",
            padx=10
        )
        self.drone_id_label.pack(side="right")

    def log_message(self, message, level="INFO"):
        """在日志区域显示消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)

        # 同时输出到logger
        if level == "ERROR":
            logger.error(message)
        elif level == "WARNING":
            logger.warning(message)
        else:
            logger.info(message)

    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)

    def update_status(self, status):
        """更新状态栏"""
        self.status_label.config(text=f"状态: {status}")

    def set_current_drone(self, drone_id: Optional[int]):
        """更新状态栏右侧的当前无人机ID显示。传入None清除显示。"""
        try:
            if drone_id is None or not getattr(self, 'drone', None):
                text = "当前无人机: —"
            else:
                text = f"当前无人机: ID={int(drone_id)}"
            # 确保在主线程更新GUI
            if hasattr(self, 'root'):
                try:
                    self.root.after(0, lambda: self.drone_id_label.config(text=text))
                except Exception:
                    # 备用：直接配置
                    self.drone_id_label.config(text=text)
            else:
                self.drone_id_label.config(text=text)
        except Exception:
            # 忽略任何更新错误（防御性处理）
            pass

    def run_in_thread(self, func, *args):
        """在线程中运行函数，避免阻塞GUI"""
        def wrapper():
            try:
                func(*args)
            except Exception as e:
                self.log_message(f"执行错误: {e}", "ERROR")
                messagebox.showerror("错误", f"执行失败: {e}")

        thread = threading.Thread(target=wrapper, daemon=True)
        thread.start()

    # 控制命令方法
    def init_manager(self):
        """初始化管理器"""
        # 防止重复初始化
        if getattr(self, 'manager', None):
            self.log_message("管理器已初始化，无需重复初始化", "WARNING")
            messagebox.showinfo("提示", "管理器已初始化")
            return

        def _init():
            self.log_message("正在初始化管理器...")
            # 从下拉框获取COM口，如果未选择则使用空字符串
            com_port = self.com_port_combo.get() if hasattr(self, 'com_port_combo') else ''
            baudrate = self.baudrate_entry.get()

            # 尝试将波特率转换为整数
            try:
                baudrate = int(baudrate)
            except ValueError:
                self.log_message("无效的波特率", "ERROR")
                messagebox.showerror("错误", "请输入有效的波特率")
                return

            # 使用串口创建管理器
            self.manager = create_manager_with_serial(com_port, baudrate)
            self.manager.init()
            # 初始化成功后，禁用COM口选择，防止在连接后被修改
            try:
                # manager.init() 在后台线程中执行，所有tkinter GUI更新需切换到主线程
                if hasattr(self, 'root') and hasattr(self, 'com_port_combo'):
                    # 禁用COM下拉，禁用初始化按钮，启用断开按钮
                    self.root.after(0, lambda: (
                        self.com_port_combo.config(state='disabled'),
                        getattr(self, 'btn_init', None) and self.btn_init.config(state='disabled'),
                        getattr(self, 'btn_disconnect', None) and self.btn_disconnect.config(state='normal')
                    ))
            except Exception:
                pass
            # 初始化成功后刷新无人机ID下拉列表（如果存在）
            try:
                self._populate_drone_ids()
            except Exception:
                pass

            # 新增：在主线程创建命令队列（自动创建）
            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: setattr(self, 'cmd_queue', ManagerCommandQueue(self.manager)))
                else:
                    self.cmd_queue = ManagerCommandQueue(self.manager)
            except Exception:
                pass

            self.log_message("✓ 管理器初始化成功")
            self.update_status("管理器已初始化")

        self.run_in_thread(_init)

    def _populate_com_ports(self):
        """列出系统上可用的串口，并更新下拉框。"""
        # 如果当前COM下拉已被禁用（表示串口已连接），不要刷新列表
        try:
            if hasattr(self, 'com_port_combo') and str(self.com_port_combo['state']) == 'disabled':
                return
        except Exception:
            pass

        ports = []
        if list_ports is not None:
            try:
                ports_info = list_ports.comports()
                ports = [p.device for p in ports_info]
            except Exception as e:
                self.log_message(f"列出COM口时出错: {e}", "WARNING")
        # 如果没有检测到端口，显示提示项
        if not ports:
            ports = ["(无可用COM口)"]

        # 更新combobox的值并选中第一个
        try:
            self.com_port_combo['values'] = ports
            self.com_port_combo.current(0)
        except Exception:
            # 在初始化之前可能会发生（防御性处理）
            pass

    def _populate_drone_ids(self):
        """填充无人机ID下拉框。优先使用manager提供的列表，否则使用默认1-10."""
        ids = []
        if getattr(self, 'manager', None):
            try:
                # 尝试从manager获取已知airplane id列表（接口不固定，做保护性检测）
                raw_ids = []
                if hasattr(self.manager, 'list_airplanes'):
                    raw_ids = list(self.manager.list_airplanes())
                elif hasattr(self.manager, 'get_airplanes'):
                    raw_ids = list(self.manager.get_airplanes())
                elif hasattr(self.manager, 'airplanes'):
                    try:
                        raw_ids = list(getattr(self.manager, 'airplanes').keys())
                    except Exception:
                        raw_ids = []

                # 规范化为整数，过滤到 0..15 范围，并排序去重（移除 ID 16）
                num_ids = []
                for r in raw_ids:
                    try:
                        v = int(r)
                        if 0 <= v <= 15:
                            num_ids.append(v)
                    except Exception:
                        continue

                num_ids = sorted(set(num_ids))
                ids = [str(i) for i in num_ids]
            except Exception as e:
                self.log_message(f"获取无人机列表时出错: {e}", "WARNING")

        if not ids:
            # 默认只显示 0..15（移除 16）
            ids = [str(i) for i in range(0, 16)]

        try:
            self.id_combo['values'] = ids
            # 如果当前选择值不存在于新列表中，则选中第一个
            current = self.id_combo.get()
            if not current or current not in ids:
                self.id_combo.set(ids[0])
        except Exception:
            pass

    def get_drone(self):
        """获取无人机对象"""
        def _get():
            if not self.manager:
                self.log_message("请先初始化管理器", "ERROR")
                messagebox.showwarning("警告", "请先初始化管理器")
                return

            try:
                # 从下拉框读取无人机ID
                self.drone_id = int(self.id_combo.get())
            except ValueError:
                self.log_message("无效的无人机ID", "ERROR")
                messagebox.showerror("错误", "请输入有效的无人机ID")
                return

            self.log_message(f"正在获取无人机 (ID={self.drone_id})...")
            self.drone = self.manager.get_airplane(self.drone_id)
            self.log_message(f"✓ 无人机对象获取成功 (ID={self.drone_id})")
            # 更新状态栏文本并更新右侧的当前无人机ID显示（在主线程中执行GUI更新）
            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: (
                        self.update_status(f"无人机已连接 (ID={self.drone_id})"),
                        self.set_current_drone(self.drone_id)
                    ))
                else:
                    self.update_status(f"无人机已连接 (ID={self.drone_id})")
                    self.set_current_drone(self.drone_id)
            except Exception:
                # 保护性回退
                self.update_status(f"无人机已连接 (ID={self.drone_id})")
                self.set_current_drone(self.drone_id)

        self.run_in_thread(_get)

    def check_drone(self):
        """检查无人机是否已初始化"""
        if not self.drone:
            self.log_message("请先获取无人机对象", "ERROR")
            messagebox.showwarning("警告", "请先初始化管理器并获取无人机对象")
            return False
        return True

    def arm(self):
        """对已选中的无人机发送解锁（arm）命令（广播）"""
        if not self.check_manager():
            return
        # 广播 arm
        self.broadcast_command('arm')

    def disarm(self):
        """对已选中的无人机发送上锁（disarm）命令（广播）"""
        if not self.check_manager():
            return
        # 广播 disarm
        self.broadcast_command('disarm')

    def takeoff(self):
        """起飞（广播到已选中的所有无人机）"""
        if not self.check_manager():
            return

        try:
            height = int(self.takeoff_height.get())
        except ValueError:
            self.log_message("无效的起飞高度", "ERROR")
            messagebox.showerror("错误", "请输入有效的起飞高度(cm)")
            return

        # 直接广播（非阻塞）
        self.broadcast_command('takeoff', height, retries=3)

    def land(self):
        """降落（广播到已选中的所有无人机）"""
        if not self.check_manager():
            return
        self.broadcast_command('land', retries=3)

    def forward(self):
        """前进（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('forward', distance)

    def back(self):
        """后退（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('back', distance)

    def left(self):
        """左移（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('left', distance)

    def right(self):
        """右移（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('right', distance)

    def up(self):
        """上升（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('up', distance)

    def down(self):
        """下降（广播）"""
        if not self.check_manager():
            return
        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return
        self.broadcast_command('down', distance)

    def goto(self):
        """飞往目标点"""
        if not self.check_manager():
            return

        try:
            x = int(self.goto_x.get())
            y = int(self.goto_y.get())
            z = int(self.goto_z.get())
        except ValueError:
            self.log_message("无效的坐标值", "ERROR")
            messagebox.showerror("错误", "请输入有效的坐标值(cm)")
            return

        self.broadcast_command('goto', x, y, z)

    def set_led(self):
        """设置常亮模式"""
        if not self.check_manager():
            return
        try:
            r = int(self.light_r.get())
            g = int(self.light_g.get())
            b = int(self.light_b.get())
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")
            messagebox.showerror("错误", "请输入有效的RGB值(0-255)")
            return
        self.broadcast_command('led', r, g, b)

    def set_breathe(self):
        """设置呼吸灯模式"""
        if not self.check_manager():
            return
        try:
            r = int(self.light_r.get())
            g = int(self.light_g.get())
            b = int(self.light_b.get())
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")
            messagebox.showerror("错误", "请输入有效的RGB值(0-255)")
            return
        self.broadcast_command('bln', r, g, b)

    def set_rainbow(self):
        """设置彩虹灯模式"""
        if not self.check_manager():
            return
        try:
            r = int(self.light_r.get())
            g = int(self.light_g.get())
            b = int(self.light_b.get())
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")
            messagebox.showerror("错误", "请输入有效的RGB值(0-255)")
            return
        self.broadcast_command('rainbow', r, g, b)

    def set_preset_color(self, r, g, b):
        """设置预设颜色"""
        # 更新输入框的值
        self.light_r.delete(0, tk.END)
        self.light_r.insert(0, str(r))
        self.light_g.delete(0, tk.END)
        self.light_g.insert(0, str(g))
        self.light_b.delete(0, tk.END)
        self.light_b.insert(0, str(b))

        # 更新颜色预览
        self.update_color_preview()

        # 如果无人机已连接，直接设置颜色
        if not self.check_manager():
            return
        self.broadcast_command('led', r, g, b)

    def update_color_preview(self, event=None):
        """更新颜色预览框（在 RGB 输入变化时调用）。"""
        try:
            r = int(self.light_r.get())
            g = int(self.light_g.get())
            b = int(self.light_b.get())
            # 限制范围0-255
            r = max(0, min(255, r))
            g = max(0, min(255, g))
            b = max(0, min(255, b))
            color = f"#{r:02X}{g:02X}{b:02X}"
            try:
                self.color_preview.config(bg=color)
            except Exception:
                pass
        except Exception:
            # 如果输入不是整数，忽略并不更新预览
            pass

    def set_flight_mode(self, mode):
        """设置飞行模式"""
        if not self.check_manager():
            return
        mode_names = {1: "常规模式", 2: "巡线模式", 3: "跟随模式"}
        mode_name = mode_names.get(mode, "未知模式")
        self.broadcast_command('airplane_mode', mode)

    def rotate_cw(self):
        """顺时针旋转指定角度（从界面读取）"""
        if not self.check_manager():
            return
        try:
            degree = int(self.rotate_angle.get())
        except Exception:
            self.log_message("无效的旋转角度", "ERROR")
            messagebox.showerror("错误", "请输入有效的旋转角度(整数)")
            return
        degree = max(0, min(360, degree))
        # 广播 cw/rotate 命令（优先 cw）
        self.broadcast_command('cw', degree)

    def rotate_ccw(self):
        """逆时针旋转指定角度（从界面读取）"""
        if not self.check_manager():
            return
        try:
            degree = int(self.rotate_angle.get())
        except Exception:
            self.log_message("无效的旋转角度", "ERROR")
            messagebox.showerror("错误", "请输入有效的旋转角度(整数)")
            return
        degree = max(0, min(360, degree))
        self.broadcast_command('ccw', degree)

    def flip_forward(self):
        """前翻 - 广播到已选无人机"""
        if not self.check_manager():
            return
        self.broadcast_command('flip_forward')

    def flip_back(self):
        """后翻 - 广播到已选无人机"""
        if not self.check_manager():
            return
        self.broadcast_command('flip_back')

    def flip_left(self):
        """左翻 - 广播到已选无人机"""
        if not self.check_manager():
            return
        self.broadcast_command('flip_left')

    def flip_right(self):
        """右翻 - 广播到已选无人机"""
        if not self.check_manager():
            return
        self.broadcast_command('flip_right')

    def toggle_heartbeat(self):
        """切换心跳包发送"""
        if not self.manager:
            self.log_message("请先初始化管理器", "WARNING")
            self.heartbeat_var.set(True)  # 重置为默认值
            return

        enabled = self.heartbeat_var.get()
        if enabled:
            self.manager.enable_heartbeat()
            self.log_message("✓ 已启用心跳包发送")
            self.heartbeat_indicator.config(fg="#4CAF50")  # 绿色
        else:
            self.manager.disable_heartbeat()
            self.log_message("✓ 已禁用心跳包发送")
            self.heartbeat_indicator.config(fg="#FF0000")  # 红色

    def disconnect_and_reset(self):
        """断开串口连接并重置所有状态"""
        def _disconnect():
            self.log_message("正在断开连接并重置状态...")

            # 在开始断开时立即禁用断开按钮，防止重复点击
            try:
                if hasattr(self, 'root') and hasattr(self, 'btn_disconnect'):
                    self.root.after(0, lambda: self.btn_disconnect.config(state='disabled'))
            except Exception:
                pass

            # 停止管理器（这会关闭串口）
            if self.manager:
                try:
                    self.manager.stop()
                    self.log_message("✓ 串口已断开")
                except Exception as e:
                    self.log_message(f"断开串口时出错: {e}", "ERROR")

            # 停止命令队列
            if getattr(self, 'cmd_queue', None):
                try:
                    self.cmd_queue.stop()
                except Exception:
                    pass

            # 重置对象引用
            self.manager = None
            self.drone = None

            # 清除当前无人机显示
            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: self.set_current_drone(None))
                else:
                    self.set_current_drone(None)
            except Exception:
                pass

            # 重置心跳包状态
            self.heartbeat_var.set(True)  # 恢复默认启用
            self.heartbeat_indicator.config(fg="#4CAF50")  # 绿色

            # 更新状态栏
            self.update_status("未初始化")

            # 重新启用COM口选择（在主线程中执行GUI更新）
            try:
                if hasattr(self, 'root') and hasattr(self, 'com_port_combo'):
                    # 恢复COM下拉、启用初始化按钮并禁用断开按钮
                    self.root.after(0, lambda: (
                        self.com_port_combo.config(state='readonly'),
                        getattr(self, 'btn_init', None) and self.btn_init.config(state='normal'),
                        getattr(self, 'btn_disconnect', None) and self.btn_disconnect.config(state='disabled')
                    ))
            except Exception:
                pass

            self.log_message("✓ 所有状态已重置")
            messagebox.showinfo("提示", "已断开连接并重置所有状态")

        self.run_in_thread(_disconnect)

    def _cleanup_manager(self):
        """清理管理器资源（同步执行）"""
        if self.manager:
            try:
                logger.info("正在停止管理器...")
                self.manager.stop()
                logger.info("✓ 管理器已停止")
            except Exception as e:
                logger.error(f"停止管理器时出错: {e}")
            finally:
                self.manager = None
                self.drone = None
                # 停止命令队列（如果存在）
                if getattr(self, 'cmd_queue', None):
                    try:
                        self.cmd_queue.stop()
                    except Exception:
                        pass
                # 清除当前无人机显示
                try:
                    if hasattr(self, 'root'):
                        self.root.after(0, lambda: self.set_current_drone(None))
                    else:
                        self.set_current_drone(None)
                except Exception:
                    pass
                # 在清理时恢复COM口选择并调整按钮状态为初始状态（启用初始化，禁用断开）
                try:
                    if hasattr(self, 'com_port_combo'):
                        if hasattr(self, 'root'):
                            self.root.after(0, lambda: (
                                self.com_port_combo.config(state='readonly'),
                                getattr(self, 'btn_init', None) and self.btn_init.config(state='normal'),
                                getattr(self, 'btn_disconnect', None) and self.btn_disconnect.config(state='disabled')
                            ))
                        else:
                            self.com_port_combo.config(state='readonly')
                            getattr(self, 'btn_init', None) and self.btn_init.config(state='normal')
                            getattr(self, 'btn_disconnect', None) and self.btn_disconnect.config(state='disabled')
                except Exception:
                    pass

    def on_closing(self):
        """关闭窗口时的处理"""
        # 同步清理资源，确保在窗口销毁前完成
        self._cleanup_manager()

        # 销毁窗口
        self.root.destroy()

        # 强制退出程序（确保所有线程都结束）
        import os
        os._exit(0)

    def _create_id_checkpanel(self, parent):
        """在指定父容器上创建无人机ID的多选复选框面板（0..15），支持折叠和两行并排显示。"""
        # 使用 LabelFrame 包裹，便于折叠与样式一致
        panel = ttk.LabelFrame(parent, text="无人机列表（多选）", padding=4)
        panel.pack(fill="x", pady=5)

        # 顶部 header：说明文字 + 折叠按钮
        header = tk.Frame(panel)
        header.pack(fill="x")
        tk.Label(header, text="选择无人机 ID:", font=("Arial", 10)).pack(side="left", padx=4)

        # 折叠状态标志与折叠/展开按钮
        self._id_check_visible = False  # 默认折叠，节省空间

        def _toggle_visibility():
            self._id_check_visible = not self._id_check_visible
            if self._id_check_visible:
                inner_frame.pack(fill="x", padx=2, pady=4)
                toggle_btn.config(text="折叠")
            else:
                inner_frame.pack_forget()
                toggle_btn.config(text="展开")

        toggle_btn = tk.Button(header, text="展开", command=_toggle_visibility,
                               bg="#FFC107", fg="black", font=("Arial", 9, "bold"), width=8)
        toggle_btn.pack(side="right", padx=4)

        # 内部框架，用于放置复选框（初始不显示）
        inner_frame = tk.Frame(panel)
        # inner_frame.pack(fill="x", padx=2, pady=4)  # 不默认显示

        # 创建两行的网格布局（0..15，共16项），分为两行各8列
        cols = 8  # 两行分布：8 + 8 = 16
        for i in range(16):
            var = tk.IntVar()
            chk = tk.Checkbutton(
                inner_frame,
                text=str(i),
                variable=var,
                font=("Arial", 9),
                onvalue=1, offvalue=0,
                command=lambda i=i, var=var: self.toggle_id_selection(i, var)
            )
            r = i // cols
            c = i % cols
            chk.grid(row=r, column=c, sticky="w", padx=2, pady=2)
            self.id_check_vars[i] = var

        # 操作按钮行（全选/反选 与 选中当前ID）
        btn_frame = tk.Frame(inner_frame)
        # 放在第 2 行（0/1 为复选框两行），按钮占据整个列跨度
        btn_frame.grid(row=2, column=0, columnspan=cols, pady=(6, 0))

        def toggle_select_all():
            any_unchecked = any(var.get() == 0 for var in self.id_check_vars.values())
            for var in self.id_check_vars.values():
                var.set(1 if any_unchecked else 0)

        def select_current_id():
            try:
                cur = int(self.id_combo.get())
            except Exception:
                return
            for i, var in self.id_check_vars.items():
                var.set(1 if i == cur else 0)

        tk.Button(btn_frame, text="全选/反选", command=toggle_select_all,
                  bg="#8BC34A", fg="white", font=("Arial", 9), width=10).pack(side="left", padx=4)
        tk.Button(btn_frame, text="选中当前ID", command=select_current_id,
                  bg="#03A9F4", fg="white", font=("Arial", 9), width=12).pack(side="left", padx=4)

        # 小提示：默认折叠，用户可以点击展开查看并选择
        tk.Label(panel, text="(折叠) 点击展开查看并选择多个无人机", font=("Arial", 8), fg="#666666").pack(fill="x", padx=4, pady=(2, 4))

    def toggle_id_selection(self, drone_id, var):
        """处理单个ID复选框的选择或取消选择（仅做选择管理，不自动发送命令）"""
        # 仅记录选择/取消，不进行自动命令发送
        # 若需在选中变化时立即反映到界面或触发其他逻辑，可在此处加入事件通知
        return

    def check_manager(self):
        """确保 manager 已初始化（用于广播场景）。"""
        if not self.manager:
            self.log_message("请先初始化管理器", "ERROR")
            messagebox.showwarning("警告", "请先初始化管理器")
            return False
        return True

    def get_selected_drone_ids(self):
        """返回当前被勾选的无人机ID列表（整数）。"""
        return [i for i, var in self.id_check_vars.items() if var.get()]

    def broadcast_command(self, command_name, *args, retries=1, **kwargs):
        """将同一命令广播发送到所有已选中的无人机（非阻塞）。

        任务会提交到 ManagerCommandQueue，执行时对串口写入进行加锁以避免冲突。
        """
        if not self.check_manager():
            return

        ids = self.get_selected_drone_ids()
        if not ids:
            self.log_message("未选择任何无人机（请展开列表并选择）", "WARNING")
            return

        # 延迟创建队列（如果尚未创建）
        if not getattr(self, 'cmd_queue', None):
            try:
                self.cmd_queue = ManagerCommandQueue(self.manager)
            except Exception as e:
                self.log_message(f"无法创建命令队列: {e}", "ERROR")
                return

        for did in ids:
            def make_cb(d=did, cmd=command_name):
                def cb(success, exc):
                    # 在主线程更新 UI 日志
                    try:
                        if hasattr(self, 'root'):
                            self.root.after(0, lambda: self.log_message(f"{'✓' if success else '✗'} 无人机 {d} 执行 {cmd} {'成功' if success else '失败: '+str(exc)}", "INFO" if success else "ERROR"))
                        else:
                            self.log_message(f"{'✓' if success else '✗'} 无人机 {d} 执行 {cmd}", "INFO" if success else "ERROR")
                    except Exception:
                        pass
                return cb

            task = CommandTask(drone_id=did, command=command_name, args=args, kwargs=kwargs, retries=retries, on_done=make_cb())
            self.cmd_queue.enqueue(task)

        self.log_message(f"已向 {len(ids)} 台无人机广播命令: {command_name}")

    def set_detect_preset(self, l_min, l_max, a_min, a_max, b_min, b_max):
        """设置色块检测预设并更新输入框。"""
        try:
            self.detect_l_min.delete(0, tk.END)
            self.detect_l_min.insert(0, str(l_min))
            self.detect_l_max.delete(0, tk.END)
            self.detect_l_max.insert(0, str(l_max))
            self.detect_a_min.delete(0, tk.END)
            self.detect_a_min.insert(0, str(a_min))
            self.detect_a_max.delete(0, tk.END)
            self.detect_a_max.insert(0, str(a_max))
            self.detect_b_min.delete(0, tk.END)
            self.detect_b_min.insert(0, str(b_min))
            self.detect_b_max.delete(0, tk.END)
            self.detect_b_max.insert(0, str(b_max))
            self.log_message("已填充色块检测预设")
        except Exception as e:
            self.log_message(f"设置色块检测预设出错: {e}", "ERROR")

    def apply_color_detect(self):
        """读取 LAB 值并广播到已选无人机，调用各无人机的 set_color_detect_mode 方法（非阻塞）。"""
        if not self.check_manager():
            return

        try:
            l_min = int(self.detect_l_min.get())
            l_max = int(self.detect_l_max.get())
            a_min = int(self.detect_a_min.get())
            a_max = int(self.detect_a_max.get())
            b_min = int(self.detect_b_min.get())
            b_max = int(self.detect_b_max.get())
        except ValueError:
            self.log_message("无效的LAB值", "ERROR")
            messagebox.showerror("错误", "请输入有效的LAB值")
            return

        # 广播到已选无人机
        self.broadcast_command('set_color_detect_mode', l_min, l_max, a_min, a_max, b_min, b_max)
        self.log_message(f"已广播色块检测设置 L({l_min}-{l_max}) A({a_min}-{a_max}) B({b_min}-{b_max})")

def main():
    """程序入口：创建 Tk 应用并运行 DroneControlGUI 界面。"""
    root = tk.Tk()
    app = DroneControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
