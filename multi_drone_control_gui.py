"""
多无人机控制GUI程序 - 支持同时控制多台无人机
可以同时向多个无人机发送相同的指令，也可以向不同无人机发送不同的指令
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import logging
from datetime import datetime
from typing import Optional, Dict, List, Set
import time
import os
import io
from owl2.airplane_manager_owl02 import create_manager_with_serial, AirplaneOwl02
try:
    from PIL import Image, ImageTk
except ImportError:
    Image = None
    ImageTk = None
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 命令任务管理
from dataclasses import dataclass, field
from typing import Any, Callable, Tuple
import concurrent.futures

@dataclass
class CommandTask:
    """表示要发送给单台无人机的命令任务"""
    drone_id: int
    command: str
    args: Tuple[Any, ...] = field(default_factory=tuple)
    kwargs: Dict[str, Any] = field(default_factory=dict)
    retries: int = 1
    on_done: Optional[Callable[[bool, Optional[Exception]], None]] = None

class ManagerCommandQueue:
    """并发命令队列：使用线程池并发处理任务，但对串口写操作使用锁序列化"""
    def __init__(self, manager, max_workers: int = 16):
        self.manager = manager
        self._write_lock = threading.Lock()
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
        self._shutdown = False

    def enqueue(self, task: CommandTask):
        if self._shutdown:
            return
        self._executor.submit(self._process_task, task)

    def _process_task(self, task: CommandTask):
        """在独立线程中处理单个任务"""
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

                with self._write_lock:
                    if airplane is not None and hasattr(airplane, task.command):
                        getattr(airplane, task.command)(*task.args, **task.kwargs)
                    # elif hasattr(self.manager, 'send_command'):
                    #     self.manager.send_command(task.drone_id, task.command, *task.args, **task.kwargs)
                    else:
                        raise AttributeError(f"无人机对象或管理器不支持命令 {task.command}")

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

    def stop(self, wait=True):
        self._shutdown = True
        try:
            self._executor.shutdown(wait=wait)
        except Exception:
            pass


class MultiDroneControlGUI:
    """多无人机控制GUI类 - 支持同时控制多台无人机"""

    def __init__(self, root):
        self.root = root
        self.root.title("多无人机协同控制系统")
        self.root.geometry("1500x1000")

        self.manager = None
        self.cmd_queue: Optional[ManagerCommandQueue] = None

        # 无人机状态管理
        self.drone_panels: Dict[int, Dict] = {}  # 存储每个无人机的UI组件
        self.global_selection: Set[int] = set()  # 全局选中的无人机ID

        # 照片接收相关状态
        self.current_photo_drone_id: Optional[int] = None  # 当前拍照的无人机ID
        self.current_photo_id: Optional[int] = None  # 当前接收的照片ID
        self.photo_progress: float = 0.0  # 照片传输进度 0.0 ~ 1.0
        self.received_image: Optional[bytes] = None  # 接收到的图片数据
        self.photo_tk_image = None  # 用于显示的Tk图片对象

        self.setup_ui()

    def setup_ui(self):
        """设置界面布局"""
        # 标题
        title_label = tk.Label(
            self.root,
            text="多无人机协同控制系统",
            font=("Arial", 18, "bold"),
            pady=10,
            bg="#2C3E50",
            fg="white"
        )
        title_label.pack(fill="x")

        # 主容器 - 分为左中右三部分
        main_container = tk.Frame(self.root)
        main_container.pack(fill="both", expand=True, padx=10, pady=5)

        # 左侧：初始化和全局控制（上下布局）
        left_section = tk.Frame(main_container)
        left_section.pack(side="left", fill="both", expand=False, padx=(0, 5))

        # 中间：无人机面板和日志（上下布局）
        middle_section = tk.Frame(main_container)
        middle_section.pack(side="left", fill="both", expand=True, padx=(5, 5))

        # 右侧：照片控制面板
        right_section = tk.Frame(main_container)
        right_section.pack(side="right", fill="both", expand=False, padx=(5, 0))

        # ==================== 左侧布局（上下） ====================
        # 上部：初始化区域
        init_frame = ttk.LabelFrame(left_section, text="系统初始化", padding=10)
        init_frame.pack(side="top", fill="x", pady=(0, 5))

        self._create_init_panel(init_frame)

        # 下部：全局控制区域
        global_control_frame = ttk.LabelFrame(left_section, text="全局控制 (对所有选中的无人机)", padding=10)
        global_control_frame.pack(side="top", fill="both", expand=True, pady=(5, 0))

        self._create_global_control_panel(global_control_frame)

        # ==================== 中间布局（上下） ====================
        # 上部：无人机面板区域（可滚动）
        drones_panel_frame = ttk.LabelFrame(middle_section, text="无人机控制面板", padding=10)
        drones_panel_frame.pack(side="top", fill="both", expand=True, pady=(0, 5))

        self._create_drones_panel(drones_panel_frame)

        # 下部：日志输出区域
        log_frame = ttk.LabelFrame(middle_section, text="系统日志", padding=10)
        log_frame.pack(side="top", fill="both", expand=False, pady=(5, 0))

        self._create_log_panel(log_frame)

        # ==================== 右侧布局：照片面板 ====================
        photo_frame = ttk.LabelFrame(right_section, text="照片拍摄与接收", padding=10)
        photo_frame.pack(side="top", fill="both", expand=True)

        self._create_photo_panel(photo_frame)

        # 底部状态栏
        self._create_status_bar()

    def _create_init_panel(self, parent):
        """创建初始化面板"""
        # COM口配置
        com_frame = tk.Frame(parent)
        com_frame.pack(fill="x", pady=5)

        tk.Label(com_frame, text="COM口:").pack(side="left", padx=5)
        self.com_port_combo = ttk.Combobox(com_frame, width=12, state="readonly")
        self._populate_com_ports()
        self.com_port_combo.pack(side="left", padx=5)

        tk.Button(com_frame, text="刷新", command=self._populate_com_ports, width=6).pack(side="left", padx=5)

        tk.Label(com_frame, text="波特率:").pack(side="left", padx=5)
        self.baudrate_entry = tk.Entry(com_frame, width=10)
        self.baudrate_entry.insert(0, "921600")
        self.baudrate_entry.pack(side="left", padx=5)

        # 初始化按钮
        self.btn_init = tk.Button(
            parent,
            text="初始化系统",
            command=self.init_manager,
            bg="#27AE60",
            fg="white",
            font=("Arial", 11, "bold"),
            height=1
        )
        self.btn_init.pack(fill="x", pady=2)

        # 断开连接按钮
        self.btn_disconnect = tk.Button(
            parent,
            text="断开连接",
            command=self.disconnect_and_reset,
            bg="#E74C3C",
            fg="white",
            font=("Arial", 11, "bold"),
            height=1,
            state='disabled'
        )
        self.btn_disconnect.pack(fill="x", pady=2)

        # 无人机数量配置
        drone_count_frame = tk.Frame(parent)
        drone_count_frame.pack(fill="x", pady=2)

        tk.Label(drone_count_frame, text="无人机数量:").pack(side="left", padx=5)
        self.drone_count_spinbox = tk.Spinbox(drone_count_frame, from_=1, to=16, width=8)
        self.drone_count_spinbox.delete(0, tk.END)
        self.drone_count_spinbox.insert(0, "3")
        self.drone_count_spinbox.pack(side="left", padx=5)

        tk.Button(
            drone_count_frame,
            text="生成控制面板",
            command=self.generate_drone_panels,
            bg="#3498DB",
            fg="white",
            font=("Arial", 9, "bold")
        ).pack(side="left", padx=5)

        # 心跳包控制
        heartbeat_frame = tk.Frame(parent)
        heartbeat_frame.pack(fill="x", pady=2)

        self.heartbeat_var = tk.BooleanVar(value=True)
        self.heartbeat_checkbox = tk.Checkbutton(
            heartbeat_frame,
            text="启用心跳包",
            variable=self.heartbeat_var,
            command=self.toggle_heartbeat,
            font=("Arial", 10)
        )
        self.heartbeat_checkbox.pack(side="left", padx=5)

        self.heartbeat_indicator = tk.Label(
            heartbeat_frame,
            text="●",
            fg="#27AE60",
            font=("Arial", 16)
        )
        self.heartbeat_indicator.pack(side="left", padx=5)

    def _create_global_control_panel(self, parent):
        """创建全局控制面板"""
        # 全选/取消全选按钮
        selection_frame = tk.Frame(parent)
        selection_frame.pack(fill="x", pady=2)

        tk.Button(
            selection_frame,
            text="全选",
            command=self.select_all_drones,
            bg="#16A085",
            fg="white",
            font=("Arial", 9, "bold"),
            width=10
        ).pack(side="left", padx=2)

        tk.Button(
            selection_frame,
            text="取消全选",
            command=self.deselect_all_drones,
            bg="#95A5A6",
            fg="white",
            font=("Arial", 9, "bold"),
            width=10
        ).pack(side="left", padx=2)

        self.selected_count_label = tk.Label(
            selection_frame,
            text="已选中: 0 架",
            font=("Arial", 10, "bold"),
            fg="#E74C3C"
        )
        self.selected_count_label.pack(side="left", padx=5)

        # 基本控制按钮
        basic_control_frame = ttk.LabelFrame(parent, text="基本控制", padding=3)
        basic_control_frame.pack(fill="x", pady=2)

        row1 = tk.Frame(basic_control_frame)
        row1.pack(fill="x", pady=1)

        tk.Button(
            row1, text="解锁 (Arm)", command=lambda: self.global_command('arm'),
            bg="#F39C12", fg="white", font=("Arial", 9, "bold"), width=12, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row1, text="上锁 (Disarm)", command=lambda: self.global_command('disarm'),
            bg="#7F8C8D", fg="white", font=("Arial", 9, "bold"), width=12, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        row2 = tk.Frame(basic_control_frame)
        row2.pack(fill="x", pady=1)

        tk.Label(row2, text="高度(cm):").pack(side="left", padx=2)
        self.global_height = tk.Entry(row2, width=8)
        self.global_height.insert(0, "100")
        self.global_height.pack(side="left", padx=2)

        tk.Button(
            row2, text="起飞", command=self.global_takeoff,
            bg="#27AE60", fg="white", font=("Arial", 9, "bold"), width=10, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row2, text="降落", command=lambda: self.global_command('land'),
            bg="#E74C3C", fg="white", font=("Arial", 9, "bold"), width=10, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        row3 = tk.Frame(basic_control_frame)
        row3.pack(fill="x", pady=1)

        tk.Label(row3, text="目标高度(cm):").pack(side="left", padx=2)
        self.global_target_height = tk.Entry(row3, width=8)
        self.global_target_height.insert(0, "150")
        self.global_target_height.pack(side="left", padx=2)

        tk.Button(
            row3, text="设置高度", command=self.global_set_height,
            bg="#9B59B6", fg="white", font=("Arial", 9, "bold"), width=10, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row3, text="悬停", command=lambda: self.global_command('hover'),
            bg="#3498DB", fg="white", font=("Arial", 9, "bold"), width=10, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        # 移动控制
        move_frame = ttk.LabelFrame(parent, text="编队移动", padding=3)
        move_frame.pack(fill="x", pady=2)

        distance_row = tk.Frame(move_frame)
        distance_row.pack(fill="x", pady=1)

        tk.Label(distance_row, text="移动距离(cm):").pack(side="left", padx=2)
        self.global_distance = tk.Entry(distance_row, width=8)
        self.global_distance.insert(0, "50")
        self.global_distance.pack(side="left", padx=2)

        direction_grid = tk.Frame(move_frame)
        direction_grid.pack(pady=1)

        # 上升
        tk.Button(
            direction_grid, text="↑ 上升", command=self.global_up,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=0, column=1, padx=1, pady=1)

        # 前进
        tk.Button(
            direction_grid, text="↑ 前进", command=self.global_forward,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=1, column=1, padx=1, pady=1)

        # 左右
        tk.Button(
            direction_grid, text="← 左移", command=self.global_left,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=2, column=0, padx=1, pady=1)

        tk.Button(
            direction_grid, text="→ 右移", command=self.global_right,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=2, column=2, padx=1, pady=1)

        # 后退
        tk.Button(
            direction_grid, text="↓ 后退", command=self.global_back,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=3, column=1, padx=1, pady=1)

        # 下降
        tk.Button(
            direction_grid, text="↓ 下降", command=self.global_down,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=4, column=1, padx=1, pady=1)

        # Goto定点飞行控制
        goto_frame = ttk.LabelFrame(parent, text="编队定点飞行 (Goto)", padding=3)
        goto_frame.pack(fill="x", pady=2)

        coords_frame = tk.Frame(goto_frame)
        coords_frame.pack(fill="x", pady=1)

        tk.Label(coords_frame, text="X(cm):").pack(side="left", padx=1)
        self.global_goto_x = tk.Entry(coords_frame, width=8)
        self.global_goto_x.insert(0, "100")
        self.global_goto_x.pack(side="left", padx=1)

        tk.Label(coords_frame, text="Y(cm):").pack(side="left", padx=1)
        self.global_goto_y = tk.Entry(coords_frame, width=8)
        self.global_goto_y.insert(0, "100")
        self.global_goto_y.pack(side="left", padx=1)

        tk.Label(coords_frame, text="Z(cm):").pack(side="left", padx=1)
        self.global_goto_z = tk.Entry(coords_frame, width=8)
        self.global_goto_z.insert(0, "150")
        self.global_goto_z.pack(side="left", padx=1)

        tk.Button(
            goto_frame, text="飞往目标点", command=self.global_goto,
            bg="#673AB7", fg="white", font=("Arial", 9, "bold"), width=15, height=1
        ).pack(pady=1)

        # 灯光控制
        light_frame = ttk.LabelFrame(parent, text="编队灯光", padding=3)
        light_frame.pack(fill="x", pady=2)

        rgb_frame = tk.Frame(light_frame)
        rgb_frame.pack(fill="x", pady=1)

        tk.Label(rgb_frame, text="R:").pack(side="left", padx=1)
        self.global_r = tk.Entry(rgb_frame, width=5)
        self.global_r.insert(0, "255")
        self.global_r.pack(side="left", padx=1)

        tk.Label(rgb_frame, text="G:").pack(side="left", padx=1)
        self.global_g = tk.Entry(rgb_frame, width=5)
        self.global_g.insert(0, "0")
        self.global_g.pack(side="left", padx=1)

        tk.Label(rgb_frame, text="B:").pack(side="left", padx=1)
        self.global_b = tk.Entry(rgb_frame, width=5)
        self.global_b.insert(0, "0")
        self.global_b.pack(side="left", padx=1)

        light_btn_frame = tk.Frame(light_frame)
        light_btn_frame.pack(fill="x", pady=1)

        tk.Button(
            light_btn_frame, text="常亮", command=self.global_led,
            bg="#F39C12", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=1, expand=True, fill="x")

        tk.Button(
            light_btn_frame, text="呼吸", command=self.global_breathe,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=1, expand=True, fill="x")

        tk.Button(
            light_btn_frame, text="彩虹", command=self.global_rainbow,
            bg="#E91E63", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=1, expand=True, fill="x")

        # OpenMV控制
        openmv_frame = ttk.LabelFrame(parent, text="OpenMV视觉控制", padding=3)
        openmv_frame.pack(fill="x", pady=2)

        mode_row = tk.Frame(openmv_frame)
        mode_row.pack(fill="x", pady=1)

        tk.Label(mode_row, text="识别模式:").pack(side="left", padx=2)
        self.openmv_mode = tk.Spinbox(mode_row, from_=1, to=3, width=8)
        self.openmv_mode.delete(0, tk.END)
        self.openmv_mode.insert(0, "1")
        self.openmv_mode.pack(side="left", padx=2)
        tk.Label(mode_row, text="(1常规 2巡线 3跟随)", font=("Arial", 8)).pack(side="left", padx=2)

        tk.Button(
            openmv_frame, text="设置OpenMV模式", command=self.global_set_openmv_mode,
            bg="#FF5722", fg="white", font=("Arial", 9, "bold"), width=20, height=1
        ).pack(pady=1)

        cmd_row = tk.Frame(openmv_frame)
        cmd_row.pack(fill="x", pady=1)

        tk.Label(cmd_row, text="视觉命令:").pack(side="left", padx=1)
        self.openmv_cmd = tk.Spinbox(cmd_row, from_=0, to=3, width=5)
        self.openmv_cmd.delete(0, tk.END)
        self.openmv_cmd.insert(0, "0")
        self.openmv_cmd.pack(side="left", padx=1)
        tk.Label(cmd_row, text="(0巡线 1锁定二维码 3寻找色块)", font=("Arial", 8)).pack(side="left", padx=1)

        openmv_coords_frame = tk.Frame(openmv_frame)
        openmv_coords_frame.pack(fill="x", pady=1)

        tk.Label(openmv_coords_frame, text="X(cm):").pack(side="left", padx=1)
        self.openmv_x = tk.Entry(openmv_coords_frame, width=6)
        self.openmv_x.insert(0, "0")
        self.openmv_x.pack(side="left", padx=1)

        tk.Label(openmv_coords_frame, text="Y(cm):").pack(side="left", padx=1)
        self.openmv_y = tk.Entry(openmv_coords_frame, width=6)
        self.openmv_y.insert(0, "0")
        self.openmv_y.pack(side="left", padx=1)

        tk.Label(openmv_coords_frame, text="Z(cm):").pack(side="left", padx=1)
        self.openmv_z = tk.Entry(openmv_coords_frame, width=6)
        self.openmv_z.insert(0, "0")
        self.openmv_z.pack(side="left", padx=1)

        tk.Button(
            openmv_frame, text="执行OpenMV命令", command=self.global_go_openmv_cmd,
            bg="#FF9800", fg="white", font=("Arial", 9, "bold"), width=20, height=1
        ).pack(pady=1)

    def _create_drones_panel(self, parent):
        """创建无人机面板容器（可滚动）"""
        # 创建Canvas和滚动条
        canvas = tk.Canvas(parent, bg="white")
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)

        self.drones_container = tk.Frame(canvas, bg="white")

        # 创建canvas窗口
        canvas_window = canvas.create_window((0, 0), window=self.drones_container, anchor="nw")

        # 配置canvas滚动
        def configure_scroll_region(event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            # 调整窗口宽度以匹配canvas宽度
            canvas.itemconfig(canvas_window, width=canvas.winfo_width())

        self.drones_container.bind("<Configure>", configure_scroll_region)
        canvas.bind("<Configure>", configure_scroll_region)

        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        # 鼠标滚轮支持
        def on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", on_mousewheel)

        self.drones_canvas = canvas

    def _create_log_panel(self, parent):
        """创建日志面板"""
        self.log_text = scrolledtext.ScrolledText(
            parent,
            height=15,
            wrap=tk.WORD,
            font=("Consolas", 9)
        )
        self.log_text.pack(fill="both", expand=True)

        tk.Button(
            parent,
            text="清空日志",
            command=self.clear_log,
            bg="#95A5A6",
            fg="white",
            width=15
        ).pack(pady=5)

    def _create_status_bar(self):
        """创建状态栏"""
        status_frame = tk.Frame(self.root, bg="#34495E", height=30)
        status_frame.pack(fill="x", side="bottom")

        self.status_label = tk.Label(
            status_frame,
            text="状态: 未初始化",
            bg="#34495E",
            fg="white",
            anchor="w",
            padx=10
        )
        self.status_label.pack(side="left", fill="x", expand=True)

    def _create_photo_panel(self, parent):
        """创建照片拍摄与接收面板"""
        # 无人机选择
        drone_select_frame = tk.Frame(parent)
        drone_select_frame.pack(fill="x", pady=5)

        tk.Label(drone_select_frame, text="选择无人机ID:", font=("Arial", 10)).pack(side="left", padx=5)
        self.photo_drone_id_combo = ttk.Combobox(drone_select_frame, width=8, state="readonly")
        self.photo_drone_id_combo['values'] = [str(i) for i in range(16)]
        self.photo_drone_id_combo.current(0)
        self.photo_drone_id_combo.pack(side="left", padx=5)

        # 刷新无人机列表按钮
        tk.Button(
            drone_select_frame,
            text="刷新",
            command=self._refresh_photo_drone_list,
            bg="#3498DB",
            fg="white",
            font=("Arial", 8),
            width=5
        ).pack(side="left", padx=5)

        # 拍照按钮
        self.btn_take_photo = tk.Button(
            parent,
            text="📷 拍摄照片",
            command=self.take_photo,
            bg="#9C27B0",
            fg="white",
            font=("Arial", 12, "bold"),
            height=2
        )
        self.btn_take_photo.pack(fill="x", pady=10)

        # 清除缓存按钮
        self.btn_clear_photo_cache = tk.Button(
            parent,
            text="🗑️ 清除无人机图片缓存",
            command=self.clear_drone_photo_cache,
            bg="#E74C3C",
            fg="white",
            font=("Arial", 10, "bold"),
            height=1
        )
        self.btn_clear_photo_cache.pack(fill="x", pady=5)

        # 传输状态
        status_frame = tk.Frame(parent)
        status_frame.pack(fill="x", pady=5)

        tk.Label(status_frame, text="传输状态:", font=("Arial", 10)).pack(side="left", padx=5)
        self.photo_status_label = tk.Label(
            status_frame,
            text="空闲",
            font=("Arial", 10, "bold"),
            fg="#27AE60"
        )
        self.photo_status_label.pack(side="left", padx=5)

        # 进度条 (Bitmap风格)
        progress_frame = tk.Frame(parent)
        progress_frame.pack(fill="x", pady=5)

        tk.Label(progress_frame, text="传输进度:", font=("Arial", 10)).pack(side="left", padx=5)

        # 使用Canvas创建bitmap风格的进度条
        self.progress_canvas = tk.Canvas(progress_frame, width=200, height=20, bg="#ECF0F1", highlightthickness=1, highlightbackground="#BDC3C7")
        self.progress_canvas.pack(side="left", padx=5, fill="x", expand=True)

        self.progress_text_label = tk.Label(progress_frame, text="0%", font=("Arial", 9), width=5)
        self.progress_text_label.pack(side="left", padx=5)

        # 照片显示区域
        photo_display_frame = ttk.LabelFrame(parent, text="接收到的照片", padding=5)
        photo_display_frame.pack(fill="both", expand=True, pady=10)

        # 照片显示Label
        self.photo_display_label = tk.Label(
            photo_display_frame,
            text="暂无照片\n\n点击'拍摄照片'按钮\n开始拍摄",
            bg="#ECF0F1",
            font=("Arial", 10),
            width=30,
            height=15
        )
        self.photo_display_label.pack(fill="both", expand=True, padx=5, pady=5)

        # 保存状态
        self.photo_save_label = tk.Label(
            parent,
            text="",
            font=("Arial", 9),
            fg="#27AE60"
        )
        self.photo_save_label.pack(fill="x", pady=5)

        # 手动保存按钮
        self.btn_save_photo = tk.Button(
            parent,
            text="💾 保存照片到桌面",
            command=self.manual_save_photo,
            bg="#3498DB",
            fg="white",
            font=("Arial", 10, "bold"),
            state='disabled'
        )
        self.btn_save_photo.pack(fill="x", pady=5)

    def take_photo(self):
        """触发拍照"""
        if not self.check_manager():
            return

        try:
            drone_id = int(self.photo_drone_id_combo.get())
        except ValueError:
            self.log_message("无效的无人机ID", "ERROR")
            messagebox.showerror("错误", "请选择有效的无人机ID")
            return

        # 获取无人机对象
        try:
            airplane = self.manager.get_airplane(drone_id)
        except Exception as e:
            self.log_message(f"获取无人机 {drone_id} 失败: {e}", "ERROR")
            messagebox.showerror("错误", f"无法获取无人机 {drone_id}")
            return

        if airplane is None:
            self.log_message(f"无人机 {drone_id} 不存在", "ERROR")
            messagebox.showerror("错误", f"无人机 {drone_id} 不存在")
            return

        # 重置状态
        self.current_photo_drone_id = drone_id
        self.current_photo_id = None
        self.photo_progress = 0.0
        self.received_image = None
        self.btn_save_photo.config(state='disabled')

        # 更新UI状态
        self.photo_status_label.config(text="正在拍照...", fg="#F39C12")
        self.photo_save_label.config(text="")
        self._update_progress_bar(0.0)

        # 设置图像接收完成回调
        airplane.image_receiver.set_image_complete_callback(self._on_image_received)

        # 发送拍照命令
        def on_capture_callback(photo_id):
            if photo_id is not None:
                self.current_photo_id = photo_id
                self.root.after(0, lambda: self._update_photo_status(f"照片ID: {photo_id}, 接收中...", "#3498DB"))
                self.log_message(f"✓ 无人机 {drone_id} 开始拍照，照片ID: {photo_id}")
                # 启动进度更新定时器
                self._start_progress_monitor(airplane, photo_id)
            else:
                self.root.after(0, lambda: self._update_photo_status("拍照失败", "#E74C3C"))
                self.log_message(f"✗ 无人机 {drone_id} 拍照失败", "ERROR")

        airplane.image_receiver.capture_image(callback=on_capture_callback)
        self.log_message(f"→ 向无人机 {drone_id} 发送拍照命令")

    def _update_photo_status(self, text, color):
        """更新照片状态标签"""
        self.photo_status_label.config(text=text, fg=color)

    def _start_progress_monitor(self, airplane: AirplaneOwl02, photo_id: int):
        """启动进度监控"""
        def update_progress():
            if photo_id not in airplane.image_receiver.image_table:
                return

            image_info = airplane.image_receiver.image_table[photo_id]

            # 如果已经收到图像数据，停止监控
            if image_info.image_data:
                return

            # 计算进度
            if image_info.total_packets > 0:
                progress = len(image_info.packet_cache) / image_info.total_packets
            else:
                progress = 0.0

            self.photo_progress = progress
            self.root.after(0, lambda: self._update_progress_bar(progress))

            # 继续监控
            if progress < 1.0:
                self.root.after(100, update_progress)

        self.root.after(100, update_progress)

    def _update_progress_bar(self, progress: float):
        """更新进度条 (bitmap风格)"""
        self.progress_canvas.delete("all")

        canvas_width = self.progress_canvas.winfo_width()
        if canvas_width < 10:
            canvas_width = 200

        canvas_height = 20

        # 绘制bitmap风格的进度条 (小方块)
        block_width = 8
        block_height = 16
        block_spacing = 2
        num_blocks = (canvas_width - 4) // (block_width + block_spacing)

        filled_blocks = int(progress * num_blocks)

        for i in range(num_blocks):
            x1 = 2 + i * (block_width + block_spacing)
            y1 = 2
            x2 = x1 + block_width
            y2 = y1 + block_height

            if i < filled_blocks:
                # 已填充的块 - 绿色渐变
                color = "#27AE60" if i % 2 == 0 else "#2ECC71"
            else:
                # 未填充的块 - 灰色
                color = "#BDC3C7"

            self.progress_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="")

        # 更新百分比文本
        percent = int(progress * 100)
        self.progress_text_label.config(text=f"{percent}%")

    def _on_image_received(self, photo_id: int, image_data: bytes):
        """图像接收完成回调"""
        self.received_image = image_data
        self.photo_progress = 1.0

        # 在主线程中更新UI
        self.root.after(0, lambda: self._display_and_save_image(photo_id, image_data))

    def _display_and_save_image(self, photo_id: int, image_data: bytes):
        """显示并保存图像"""
        # 更新进度条到100%
        self._update_progress_bar(1.0)
        self._update_photo_status("接收完成!", "#27AE60")

        # 显示图片
        if Image is not None and ImageTk is not None:
            try:
                # 从bytes创建图像
                image = Image.open(io.BytesIO(image_data))

                # 调整大小以适应显示区域
                display_size = (280, 210)
                image.thumbnail(display_size, Image.Resampling.LANCZOS)

                # 转换为Tk可显示的格式
                self.photo_tk_image = ImageTk.PhotoImage(image)

                # 显示图片
                self.photo_display_label.config(image=self.photo_tk_image, text="")
            except Exception as e:
                self.log_message(f"显示图片失败: {e}", "ERROR")
                self.photo_display_label.config(text=f"图片显示失败\n{e}", image="")
        else:
            self.photo_display_label.config(text=f"照片已接收\n大小: {len(image_data)} bytes\n\n(需要PIL库才能显示图片)", image="")

        # 自动保存到桌面
        save_path = self._save_image_to_desktop(photo_id, image_data)
        if save_path:
            self.photo_save_label.config(text=f"已保存: {save_path}", fg="#27AE60")
            self.log_message(f"✓ 照片已保存到: {save_path}")
        else:
            self.photo_save_label.config(text="保存失败", fg="#E74C3C")

        # 启用手动保存按钮
        self.btn_save_photo.config(state='normal')

    def _save_image_to_desktop(self, photo_id: int, image_data: bytes) -> Optional[str]:
        """保存图片到桌面"""
        try:
            # 获取桌面路径
            desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
            if not os.path.exists(desktop_path):
                # 尝试中文桌面路径
                desktop_path = os.path.join(os.path.expanduser("~"), "桌面")
            if not os.path.exists(desktop_path):
                # 使用用户目录
                desktop_path = os.path.expanduser("~")

            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"drone_{self.current_photo_drone_id}_photo_{photo_id}_{timestamp}.jpg"
            filepath = os.path.join(desktop_path, filename)

            # 保存文件
            with open(filepath, 'wb') as f:
                f.write(image_data)

            return filepath
        except Exception as e:
            self.log_message(f"保存照片失败: {e}", "ERROR")
            return None

    def manual_save_photo(self):
        """手动保存照片到桌面"""
        if self.received_image is None:
            messagebox.showwarning("警告", "没有可保存的照片")
            return

        photo_id = self.current_photo_id if self.current_photo_id else 0
        save_path = self._save_image_to_desktop(photo_id, self.received_image)
        if save_path:
            self.photo_save_label.config(text=f"已保存: {save_path}", fg="#27AE60")
            messagebox.showinfo("保存成功", f"照片已保存到:\n{save_path}")
        else:
            messagebox.showerror("保存失败", "无法保存照片")

    def _refresh_photo_drone_list(self):
        """刷新照片面板的无人机下拉列表"""
        if not self.manager:
            # 如果没有初始化，显示默认的0-15
            self.photo_drone_id_combo['values'] = [str(i) for i in range(16)]
            return

        # 获取当前已连接/已知的无人机ID列表
        drone_ids = []
        try:
            # 优先从drone_panels获取已生成的面板ID
            if self.drone_panels:
                drone_ids = sorted(self.drone_panels.keys())
            else:
                # 默认0-15
                drone_ids = list(range(16))
        except Exception:
            drone_ids = list(range(16))

        self.photo_drone_id_combo['values'] = [str(i) for i in drone_ids]
        if drone_ids and self.photo_drone_id_combo.get() not in [str(i) for i in drone_ids]:
            self.photo_drone_id_combo.current(0)

        self.log_message(f"已刷新无人机列表: {drone_ids}")

    def clear_drone_photo_cache(self):
        """清除无人机上缓存的所有图片"""
        if not self.check_manager():
            return

        try:
            drone_id = int(self.photo_drone_id_combo.get())
        except ValueError:
            self.log_message("无效的无人机ID", "ERROR")
            messagebox.showerror("错误", "请选择有效的无人机ID")
            return

        # 获取无人机对象
        try:
            airplane = self.manager.get_airplane(drone_id)
        except Exception as e:
            self.log_message(f"获取无人机 {drone_id} 失败: {e}", "ERROR")
            messagebox.showerror("错误", f"无法获取无人机 {drone_id}")
            return

        if airplane is None:
            self.log_message(f"无人机 {drone_id} 不存在", "ERROR")
            messagebox.showerror("错误", f"无人机 {drone_id} 不存在")
            return

        # 发送清除所有图片缓存命令 (photo_id=0表示清除所有)
        airplane.image_receiver.send_msg_clear_photo(photo_id=0)
        self.log_message(f"→ 向无人机 {drone_id} 发送清除图片缓存命令")
        self._update_photo_status("已发送清除缓存命令", "#F39C12")

    def generate_drone_panels(self):
        """生成无人机控制面板"""
        try:
            count = int(self.drone_count_spinbox.get())
            if count < 1 or count > 16:
                messagebox.showerror("错误", "无人机数量必须在1-16之间")
                return
        except ValueError:
            messagebox.showerror("错误", "请输入有效的无人机数量")
            return

        # 清空现有面板
        for widget in self.drones_container.winfo_children():
            widget.destroy()
        self.drone_panels.clear()
        self.global_selection.clear()

        # 创建新面板（每行显示2个）
        for i in range(count):
            row = i // 2
            col = i % 2
            self._create_single_drone_panel(self.drones_container, i, row, col)

        self.update_selected_count()
        self.log_message(f"✓ 已生成 {count} 个无人机控制面板")

    def _create_single_drone_panel(self, parent, drone_id, row, col):
        """创建单个无人机的控制面板"""
        # 外框
        frame = ttk.LabelFrame(parent, text=f"无人机 ID={drone_id}", padding=8)
        frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

        # 配置网格权重
        parent.grid_rowconfigure(row, weight=1)
        parent.grid_columnconfigure(col, weight=1)

        # 面板字典
        panel = {
            'frame': frame,
            'selected': False
        }

        # 选择复选框和状态指示
        header_frame = tk.Frame(frame, bg="#ECF0F1")
        header_frame.pack(fill="x", pady=(0, 5))

        select_var = tk.BooleanVar(value=False)
        panel['select_var'] = select_var

        select_cb = tk.Checkbutton(
            header_frame,
            text="选中",
            variable=select_var,
            command=lambda: self.toggle_drone_selection(drone_id),
            font=("Arial", 10, "bold"),
            bg="#ECF0F1"
        )
        select_cb.pack(side="left", padx=5)

        status_indicator = tk.Label(
            header_frame,
            text="●",
            fg="#95A5A6",
            font=("Arial", 14),
            bg="#ECF0F1"
        )
        status_indicator.pack(side="left", padx=5)
        panel['status_indicator'] = status_indicator

        status_text = tk.Label(
            header_frame,
            text="待命",
            font=("Arial", 9),
            bg="#ECF0F1"
        )
        status_text.pack(side="left", padx=5)
        panel['status_text'] = status_text

        # 快捷操作按钮（两行）
        quick_frame = tk.Frame(frame)
        quick_frame.pack(fill="x", pady=3)

        # 第一行
        row1 = tk.Frame(quick_frame)
        row1.pack(fill="x", pady=2)

        tk.Button(
            row1, text="解锁", command=lambda: self.single_command(drone_id, 'arm'),
            bg="#F39C12", fg="white", font=("Arial", 8), width=6, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row1, text="上锁", command=lambda: self.single_command(drone_id, 'disarm'),
            bg="#7F8C8D", fg="white", font=("Arial", 8), width=6, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row1, text="起飞", command=lambda: self.single_takeoff(drone_id),
            bg="#27AE60", fg="white", font=("Arial", 8), width=6, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            row1, text="降落", command=lambda: self.single_command(drone_id, 'land'),
            bg="#E74C3C", fg="white", font=("Arial", 8), width=6, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        # 第二行 - 移动控制
        row2 = tk.Frame(quick_frame)
        row2.pack(fill="x", pady=2)

        panel['distance_entry'] = tk.Entry(row2, width=6)
        panel['distance_entry'].insert(0, "50")
        panel['distance_entry'].pack(side="left", padx=2)

        tk.Label(row2, text="cm", font=("Arial", 8)).pack(side="left")

        move_buttons = [
            ("↑", lambda: self.single_move(drone_id, 'up')),
            ("↓", lambda: self.single_move(drone_id, 'down')),
            ("←", lambda: self.single_move(drone_id, 'left')),
            ("→", lambda: self.single_move(drone_id, 'right')),
            ("⬆", lambda: self.single_move(drone_id, 'forward')),
            ("⬇", lambda: self.single_move(drone_id, 'back')),
        ]

        for text, cmd in move_buttons:
            tk.Button(
                row2, text=text, command=cmd,
                bg="#3498DB", fg="white", font=("Arial", 8), width=3, height=1
            ).pack(side="left", padx=1)

        # 第三行 - 灯光控制
        row3 = tk.Frame(quick_frame)
        row3.pack(fill="x", pady=2)

        panel['r_entry'] = tk.Entry(row3, width=4)
        panel['r_entry'].insert(0, "255")
        panel['r_entry'].pack(side="left", padx=1)

        panel['g_entry'] = tk.Entry(row3, width=4)
        panel['g_entry'].insert(0, "0")
        panel['g_entry'].pack(side="left", padx=1)

        panel['b_entry'] = tk.Entry(row3, width=4)
        panel['b_entry'].insert(0, "0")
        panel['b_entry'].pack(side="left", padx=1)

        tk.Button(
            row3, text="LED", command=lambda: self.single_led(drone_id),
            bg="#F39C12", fg="white", font=("Arial", 8), width=5, height=1
        ).pack(side="left", padx=1)

        tk.Button(
            row3, text="呼吸", command=lambda: self.single_breathe(drone_id),
            bg="#3498DB", fg="white", font=("Arial", 8), width=5, height=1
        ).pack(side="left", padx=1)

        tk.Button(
            row3, text="彩虹", command=lambda: self.single_rainbow(drone_id),
            bg="#E91E63", fg="white", font=("Arial", 8), width=5, height=1
        ).pack(side="left", padx=1)

        # 第四行 - Goto定点飞行控制
        row4 = tk.Frame(quick_frame)
        row4.pack(fill="x", pady=2)

        tk.Label(row4, text="Goto:", font=("Arial", 8, "bold")).pack(side="left", padx=2)

        tk.Label(row4, text="X:", font=("Arial", 8)).pack(side="left")
        panel['goto_x'] = tk.Entry(row4, width=5)
        panel['goto_x'].insert(0, "100")
        panel['goto_x'].pack(side="left", padx=1)

        tk.Label(row4, text="Y:", font=("Arial", 8)).pack(side="left")
        panel['goto_y'] = tk.Entry(row4, width=5)
        panel['goto_y'].insert(0, "100")
        panel['goto_y'].pack(side="left", padx=1)

        tk.Label(row4, text="Z:", font=("Arial", 8)).pack(side="left")
        panel['goto_z'] = tk.Entry(row4, width=5)
        panel['goto_z'].insert(0, "150")
        panel['goto_z'].pack(side="left", padx=1)

        tk.Button(
            row4, text="飞往", command=lambda: self.single_goto(drone_id),
            bg="#673AB7", fg="white", font=("Arial", 8, "bold"), width=6, height=1
        ).pack(side="left", padx=2)

        self.drone_panels[drone_id] = panel

    def toggle_drone_selection(self, drone_id):
        """切换无人机选中状态"""
        panel = self.drone_panels.get(drone_id)
        if not panel:
            return

        if panel['select_var'].get():
            self.global_selection.add(drone_id)
            panel['frame'].configure(style="Selected.TLabelframe")
            panel['status_indicator'].config(fg="#27AE60")
        else:
            self.global_selection.discard(drone_id)
            panel['status_indicator'].config(fg="#95A5A6")

        self.update_selected_count()

    def select_all_drones(self):
        """选中所有无人机"""
        for drone_id, panel in self.drone_panels.items():
            panel['select_var'].set(True)
            self.global_selection.add(drone_id)
            panel['status_indicator'].config(fg="#27AE60")
        self.update_selected_count()

    def deselect_all_drones(self):
        """取消选中所有无人机"""
        for drone_id, panel in self.drone_panels.items():
            panel['select_var'].set(False)
            panel['status_indicator'].config(fg="#95A5A6")
        self.global_selection.clear()
        self.update_selected_count()

    def update_selected_count(self):
        """更新选中数量显示"""
        count = len(self.global_selection)
        self.selected_count_label.config(text=f"已选中: {count} 架")

    # ==================== 系统管理方法 ====================

    def init_manager(self):
        """初始化管理器"""
        if self.manager:
            self.log_message("管理器已初始化", "WARNING")
            messagebox.showinfo("提示", "管理器已初始化")
            return

        def _init():
            self.log_message("正在初始化系统...")
            com_port = self.com_port_combo.get()
            baudrate = self.baudrate_entry.get()

            try:
                baudrate = int(baudrate)
            except ValueError:
                self.log_message("无效的波特率", "ERROR")
                messagebox.showerror("错误", "请输入有效的波特率")
                return

            self.manager = create_manager_with_serial(com_port, baudrate)
            self.manager.init()

            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: (
                        self.com_port_combo.config(state='disabled'),
                        self.btn_init.config(state='disabled'),
                        self.btn_disconnect.config(state='normal')
                    ))
            except Exception:
                pass

            # 创建命令队列
            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: setattr(self, 'cmd_queue', ManagerCommandQueue(self.manager)))
                else:
                    self.cmd_queue = ManagerCommandQueue(self.manager)
            except Exception:
                pass

            self.log_message("✓ 系统初始化成功")
            self.update_status("系统已初始化")

        self.run_in_thread(_init)

    def disconnect_and_reset(self):
        """断开连接并重置"""
        def _disconnect():
            self.log_message("正在断开连接...")

            if self.manager:
                try:
                    self.manager.stop()
                    self.log_message("✓ 连接已断开")
                except Exception as e:
                    self.log_message(f"断开连接时出错: {e}", "ERROR")

            if self.cmd_queue:
                try:
                    self.cmd_queue.stop()
                except Exception:
                    pass

            self.manager = None
            self.cmd_queue = None

            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: (
                        self.com_port_combo.config(state='readonly'),
                        self.btn_init.config(state='normal'),
                        self.btn_disconnect.config(state='disabled')
                    ))
            except Exception:
                pass

            self.update_status("未初始化")
            self.log_message("✓ 系统已重置")

        self.run_in_thread(_disconnect)

    def toggle_heartbeat(self):
        """切换心跳包"""
        if not self.manager:
            self.log_message("请先初始化系统", "WARNING")
            self.heartbeat_var.set(True)
            return

        enabled = self.heartbeat_var.get()
        if enabled:
            self.manager.enable_heartbeat()
            self.log_message("✓ 已启用心跳包")
            self.heartbeat_indicator.config(fg="#27AE60")
        else:
            self.manager.disable_heartbeat()
            self.log_message("✓ 已禁用心跳包")
            self.heartbeat_indicator.config(fg="#E74C3C")

    # ==================== 命令发送方法 ====================

    def check_manager(self):
        """检查管理器是否初始化"""
        if not self.manager:
            self.log_message("请先初始化系统", "ERROR")
            messagebox.showwarning("警告", "请先初始化系统")
            return False
        return True

    def single_command(self, drone_id, command, *args, retries=1, **kwargs):
        """向单个无人机发送命令"""
        if not self.check_manager():
            return

        if not self.cmd_queue:
            self.cmd_queue = ManagerCommandQueue(self.manager)

        def callback(success, exc):
            status = "✓" if success else "✗"
            msg = f"{status} 无人机 {drone_id} 执行 {command}"
            if not success and exc:
                msg += f" 失败: {exc}"

            try:
                if hasattr(self, 'root'):
                    self.root.after(0, lambda: self.log_message(msg, "INFO" if success else "ERROR"))
                else:
                    self.log_message(msg, "INFO" if success else "ERROR")
            except Exception:
                pass

        task = CommandTask(
            drone_id=drone_id,
            command=command,
            args=args,
            kwargs=kwargs,
            retries=retries,
            on_done=callback
        )
        self.cmd_queue.enqueue(task)
        self.log_message(f"→ 无人机 {drone_id}: {command}")

    def global_command(self, command, *args, retries=1, **kwargs):
        """向所有选中的无人机发送相同命令"""
        if not self.check_manager():
            return

        if not self.global_selection:
            self.log_message("未选中任何无人机", "WARNING")
            messagebox.showwarning("警告", "请先选中要控制的无人机")
            return

        if not self.cmd_queue:
            self.cmd_queue = ManagerCommandQueue(self.manager)

        for drone_id in self.global_selection:
            def make_callback(did=drone_id, cmd=command):
                def callback(success, exc):
                    status = "✓" if success else "✗"
                    msg = f"{status} 无人机 {did} 执行 {cmd}"
                    if not success and exc:
                        msg += f" 失败: {exc}"

                    try:
                        if hasattr(self, 'root'):
                            self.root.after(0, lambda: self.log_message(msg, "INFO" if success else "ERROR"))
                        else:
                            self.log_message(msg, "INFO" if success else "ERROR")
                    except Exception:
                        pass
                return callback

            task = CommandTask(
                drone_id=drone_id,
                command=command,
                args=args,
                kwargs=kwargs,
                retries=retries,
                on_done=make_callback()
            )
            self.cmd_queue.enqueue(task)

        self.log_message(f"⇒ 广播命令 {command} 到 {len(self.global_selection)} 架无人机")

    # ==================== 具体命令方法 ====================

    # 单机命令
    def single_takeoff(self, drone_id):
        """单机起飞"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                height = int(self.global_height.get())
                self.single_command(drone_id, 'takeoff', height, retries=3)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的高度值", "ERROR")

    def single_move(self, drone_id, direction):
        """单机移动"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                distance = int(panel['distance_entry'].get())
                self.single_command(drone_id, direction, distance)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的距离值", "ERROR")

    def single_led(self, drone_id):
        """单机LED控制"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                r = int(panel['r_entry'].get())
                g = int(panel['g_entry'].get())
                b = int(panel['b_entry'].get())
                self.single_command(drone_id, 'led', r, g, b)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的RGB值", "ERROR")

    def single_breathe(self, drone_id):
        """单机呼吸灯"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                r = int(panel['r_entry'].get())
                g = int(panel['g_entry'].get())
                b = int(panel['b_entry'].get())
                self.single_command(drone_id, 'bln', r, g, b)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的RGB值", "ERROR")

    def single_rainbow(self, drone_id):
        """单机彩虹灯"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                r = int(panel['r_entry'].get())
                g = int(panel['g_entry'].get())
                b = int(panel['b_entry'].get())
                self.single_command(drone_id, 'rainbow', r, g, b)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的RGB值", "ERROR")

    def single_goto(self, drone_id):
        """单机定点飞行"""
        panel = self.drone_panels.get(drone_id)
        if panel:
            try:
                x = int(panel['goto_x'].get())
                y = int(panel['goto_y'].get())
                z = int(panel['goto_z'].get())
                self.single_command(drone_id, 'goto', x, y, z)
            except ValueError:
                self.log_message(f"无人机 {drone_id}: 无效的坐标值", "ERROR")

    # 全局命令
    def global_takeoff(self):
        """全局起飞"""
        try:
            height = int(self.global_height.get())
            self.global_command('takeoff', height, retries=3)
        except ValueError:
            self.log_message("无效的起飞高度", "ERROR")
            messagebox.showerror("错误", "请输入有效的起飞高度")

    def global_forward(self):
        """全局前进"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('forward', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_back(self):
        """全局后退"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('back', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_left(self):
        """全局左移"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('left', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_right(self):
        """全局右移"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('right', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_up(self):
        """全局上升"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('up', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_down(self):
        """全局下降"""
        try:
            distance = int(self.global_distance.get())
            self.global_command('down', distance)
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")

    def global_led(self):
        """全局LED控制"""
        try:
            r = int(self.global_r.get())
            g = int(self.global_g.get())
            b = int(self.global_b.get())
            self.global_command('led', r, g, b)
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")
            messagebox.showerror("错误", "请输入有效的RGB值")

    def global_breathe(self):
        """全局呼吸灯"""
        try:
            r = int(self.global_r.get())
            g = int(self.global_g.get())
            b = int(self.global_b.get())
            self.global_command('bln', r, g, b)
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")

    def global_rainbow(self):
        """全局彩虹灯"""
        try:
            r = int(self.global_r.get())
            g = int(self.global_g.get())
            b = int(self.global_b.get())
            self.global_command('rainbow', r, g, b)
        except ValueError:
            self.log_message("无效的RGB值", "ERROR")

    def global_goto(self):
        """全局定点飞行"""
        try:
            x = int(self.global_goto_x.get())
            y = int(self.global_goto_y.get())
            z = int(self.global_goto_z.get())
            self.global_command('goto', x, y, z)
        except ValueError:
            self.log_message("无效的坐标值", "ERROR")
            messagebox.showerror("错误", "请输入有效的坐标值(cm)")

    def global_set_height(self):
        """全局设置到指定高度"""
        try:
            height = int(self.global_target_height.get())
            # 使用goto命令，x=0, y=0，只改变高度
            self.global_command('goto', 0, 0, height)
            self.log_message(f"正在设置高度到 {height}cm", "INFO")
        except ValueError:
            self.log_message("无效的高度值", "ERROR")
            messagebox.showerror("错误", "请输入有效的高度值(cm)")

    def global_set_openmv_mode(self):
        """全局设置OpenMV模式"""
        try:
            mode = int(self.openmv_mode.get())
            if mode < 1 or mode > 3:
                raise ValueError("模式必须在1-3之间")
            self.global_command('set_openmv_mode', mode)
            mode_name = {1: "常规", 2: "巡线", 3: "跟随"}
            self.log_message(f"正在设置OpenMV模式为: {mode_name.get(mode, mode)}", "INFO")
        except ValueError as e:
            self.log_message(f"无效的OpenMV模式: {e}", "ERROR")
            messagebox.showerror("错误", "请输入有效的OpenMV模式(1-3)")

    def global_go_openmv_cmd(self):
        """全局执行OpenMV命令"""
        try:
            cmd = int(self.openmv_cmd.get())
            x = int(self.openmv_x.get())
            y = int(self.openmv_y.get())
            z = int(self.openmv_z.get())
            self.global_command('go_openmv_cmd', cmd, x, y, z)
            cmd_name = {0: "巡线", 1: "锁定二维码", 3: "寻找色块"}
            self.log_message(f"正在执行OpenMV命令: {cmd_name.get(cmd, cmd)}, 坐标({x},{y},{z})", "INFO")
        except ValueError:
            self.log_message("无效的OpenMV命令参数", "ERROR")
            messagebox.showerror("错误", "请输入有效的OpenMV命令参数")

    # ==================== 辅助方法 ====================

    def _populate_com_ports(self):
        """列出可用的COM口"""
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

        if not ports:
            ports = ["(无可用COM口)"]

        try:
            self.com_port_combo['values'] = ports
            if ports:
                self.com_port_combo.current(0)
        except Exception:
            pass

    def log_message(self, message, level="INFO"):
        """记录日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"

        try:
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
        except:
            pass

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

    def run_in_thread(self, func, *args):
        """在线程中运行函数"""
        def wrapper():
            try:
                func(*args)
            except Exception as e:
                self.log_message(f"执行错误: {e}", "ERROR")
                messagebox.showerror("错误", f"执行失败: {e}")

        thread = threading.Thread(target=wrapper, daemon=True)
        thread.start()

    def on_closing(self):
        """关闭窗口处理"""
        if self.manager:
            try:
                self.manager.stop()
            except Exception as e:
                logger.error(f"停止管理器时出错: {e}")

        if self.cmd_queue:
            try:
                self.cmd_queue.stop()
            except Exception:
                pass

        self.root.destroy()
        import os
        os._exit(0)


def main():
    """程序入口"""
    root = tk.Tk()
    app = MultiDroneControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

