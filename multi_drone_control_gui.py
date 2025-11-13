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
from owl2.airplane_manager_owl02 import create_manager_with_serial, AirplaneOwl02
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
        self.root.geometry("1600x1000")

        self.manager = None
        self.cmd_queue: Optional[ManagerCommandQueue] = None

        # 无人机状态管理
        self.drone_panels: Dict[int, Dict] = {}  # 存储每个无人机的UI组件
        self.global_selection: Set[int] = set()  # 全局选中的无人机ID

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

        # 主容器 - 分为左右两部分
        main_container = tk.Frame(self.root)
        main_container.pack(fill="both", expand=True, padx=10, pady=5)

        # 左侧：初始化和全局控制（上下布局）
        left_section = tk.Frame(main_container)
        left_section.pack(side="left", fill="both", expand=False, padx=(0, 5))

        # 右侧：无人机面板和日志（上下布局）
        right_section = tk.Frame(main_container)
        right_section.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # ==================== 左侧布局（上下） ====================
        # 上部：初始化区域
        init_frame = ttk.LabelFrame(left_section, text="系统初始化", padding=10)
        init_frame.pack(side="top", fill="x", pady=(0, 5))

        self._create_init_panel(init_frame)

        # 下部：全局控制区域
        global_control_frame = ttk.LabelFrame(left_section, text="全局控制 (对所有选中的无人机)", padding=10)
        global_control_frame.pack(side="top", fill="both", expand=True, pady=(5, 0))

        self._create_global_control_panel(global_control_frame)

        # ==================== 右侧布局（上下） ====================
        # 上部：无人机面板区域（可滚动）
        drones_panel_frame = ttk.LabelFrame(right_section, text="无人机控制面板", padding=10)
        drones_panel_frame.pack(side="top", fill="both", expand=True, pady=(0, 5))

        self._create_drones_panel(drones_panel_frame)

        # 下部：日志输出区域
        log_frame = ttk.LabelFrame(right_section, text="系统日志", padding=10)
        log_frame.pack(side="top", fill="both", expand=False, pady=(5, 0))

        self._create_log_panel(log_frame)

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
            height=2
        )
        self.btn_init.pack(fill="x", pady=5)

        # 断开连接按钮
        self.btn_disconnect = tk.Button(
            parent,
            text="断开连接",
            command=self.disconnect_and_reset,
            bg="#E74C3C",
            fg="white",
            font=("Arial", 11, "bold"),
            height=2,
            state='disabled'
        )
        self.btn_disconnect.pack(fill="x", pady=5)

        # 无人机数量配置
        drone_count_frame = tk.Frame(parent)
        drone_count_frame.pack(fill="x", pady=5)

        tk.Label(drone_count_frame, text="无人机数量:").pack(side="left", padx=5)
        self.drone_count_spinbox = tk.Spinbox(drone_count_frame, from_=1, to=16, width=8)
        self.drone_count_spinbox.delete(0, tk.END)
        self.drone_count_spinbox.insert(0, "4")
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
        heartbeat_frame.pack(fill="x", pady=5)

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
        selection_frame.pack(fill="x", pady=5)

        tk.Button(
            selection_frame,
            text="全选",
            command=self.select_all_drones,
            bg="#16A085",
            fg="white",
            font=("Arial", 9, "bold"),
            width=10
        ).pack(side="left", padx=5)

        tk.Button(
            selection_frame,
            text="取消全选",
            command=self.deselect_all_drones,
            bg="#95A5A6",
            fg="white",
            font=("Arial", 9, "bold"),
            width=10
        ).pack(side="left", padx=5)

        self.selected_count_label = tk.Label(
            selection_frame,
            text="已选中: 0 架",
            font=("Arial", 10, "bold"),
            fg="#E74C3C"
        )
        self.selected_count_label.pack(side="left", padx=10)

        # 基本控制按钮
        basic_control_frame = ttk.LabelFrame(parent, text="基本控制", padding=5)
        basic_control_frame.pack(fill="x", pady=5)

        row1 = tk.Frame(basic_control_frame)
        row1.pack(fill="x", pady=2)

        tk.Button(
            row1, text="解锁 (Arm)", command=lambda: self.global_command('arm'),
            bg="#F39C12", fg="white", font=("Arial", 9, "bold"), width=12, height=2
        ).pack(side="left", padx=3, expand=True, fill="x")

        tk.Button(
            row1, text="上锁 (Disarm)", command=lambda: self.global_command('disarm'),
            bg="#7F8C8D", fg="white", font=("Arial", 9, "bold"), width=12, height=2
        ).pack(side="left", padx=3, expand=True, fill="x")

        row2 = tk.Frame(basic_control_frame)
        row2.pack(fill="x", pady=2)

        tk.Label(row2, text="高度(cm):").pack(side="left", padx=3)
        self.global_height = tk.Entry(row2, width=8)
        self.global_height.insert(0, "150")
        self.global_height.pack(side="left", padx=3)

        tk.Button(
            row2, text="起飞", command=self.global_takeoff,
            bg="#27AE60", fg="white", font=("Arial", 9, "bold"), width=10, height=2
        ).pack(side="left", padx=3, expand=True, fill="x")

        tk.Button(
            row2, text="降落", command=lambda: self.global_command('land'),
            bg="#E74C3C", fg="white", font=("Arial", 9, "bold"), width=10, height=2
        ).pack(side="left", padx=3, expand=True, fill="x")

        # 移动控制
        move_frame = ttk.LabelFrame(parent, text="编队移动", padding=5)
        move_frame.pack(fill="x", pady=5)

        distance_row = tk.Frame(move_frame)
        distance_row.pack(fill="x", pady=2)

        tk.Label(distance_row, text="移动距离(cm):").pack(side="left", padx=3)
        self.global_distance = tk.Entry(distance_row, width=8)
        self.global_distance.insert(0, "100")
        self.global_distance.pack(side="left", padx=3)

        direction_grid = tk.Frame(move_frame)
        direction_grid.pack(pady=3)

        # 上升
        tk.Button(
            direction_grid, text="↑ 上升", command=self.global_up,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=0, column=1, padx=2, pady=2)

        # 前进
        tk.Button(
            direction_grid, text="↑ 前进", command=self.global_forward,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=1, column=1, padx=2, pady=2)

        # 左右
        tk.Button(
            direction_grid, text="← 左移", command=self.global_left,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=2, column=0, padx=2, pady=2)

        tk.Button(
            direction_grid, text="→ 右移", command=self.global_right,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=2, column=2, padx=2, pady=2)

        # 后退
        tk.Button(
            direction_grid, text="↓ 后退", command=self.global_back,
            bg="#16A085", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=3, column=1, padx=2, pady=2)

        # 下降
        tk.Button(
            direction_grid, text="↓ 下降", command=self.global_down,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=10, height=1
        ).grid(row=4, column=1, padx=2, pady=2)

        # Goto定点飞行控制
        goto_frame = ttk.LabelFrame(parent, text="编队定点飞行 (Goto)", padding=5)
        goto_frame.pack(fill="x", pady=5)

        coords_frame = tk.Frame(goto_frame)
        coords_frame.pack(fill="x", pady=2)

        tk.Label(coords_frame, text="X(cm):").pack(side="left", padx=2)
        self.global_goto_x = tk.Entry(coords_frame, width=8)
        self.global_goto_x.insert(0, "100")
        self.global_goto_x.pack(side="left", padx=2)

        tk.Label(coords_frame, text="Y(cm):").pack(side="left", padx=2)
        self.global_goto_y = tk.Entry(coords_frame, width=8)
        self.global_goto_y.insert(0, "100")
        self.global_goto_y.pack(side="left", padx=2)

        tk.Label(coords_frame, text="Z(cm):").pack(side="left", padx=2)
        self.global_goto_z = tk.Entry(coords_frame, width=8)
        self.global_goto_z.insert(0, "150")
        self.global_goto_z.pack(side="left", padx=2)

        tk.Button(
            goto_frame, text="飞往目标点", command=self.global_goto,
            bg="#673AB7", fg="white", font=("Arial", 9, "bold"), width=15, height=2
        ).pack(pady=3)

        # 灯光控制
        light_frame = ttk.LabelFrame(parent, text="编队灯光", padding=5)
        light_frame.pack(fill="x", pady=5)

        rgb_frame = tk.Frame(light_frame)
        rgb_frame.pack(fill="x", pady=2)

        tk.Label(rgb_frame, text="R:").pack(side="left", padx=2)
        self.global_r = tk.Entry(rgb_frame, width=5)
        self.global_r.insert(0, "255")
        self.global_r.pack(side="left", padx=2)

        tk.Label(rgb_frame, text="G:").pack(side="left", padx=2)
        self.global_g = tk.Entry(rgb_frame, width=5)
        self.global_g.insert(0, "0")
        self.global_g.pack(side="left", padx=2)

        tk.Label(rgb_frame, text="B:").pack(side="left", padx=2)
        self.global_b = tk.Entry(rgb_frame, width=5)
        self.global_b.insert(0, "0")
        self.global_b.pack(side="left", padx=2)

        light_btn_frame = tk.Frame(light_frame)
        light_btn_frame.pack(fill="x", pady=2)

        tk.Button(
            light_btn_frame, text="常亮", command=self.global_led,
            bg="#F39C12", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            light_btn_frame, text="呼吸", command=self.global_breathe,
            bg="#3498DB", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            light_btn_frame, text="彩虹", command=self.global_rainbow,
            bg="#E91E63", fg="white", font=("Arial", 8, "bold"), width=8, height=1
        ).pack(side="left", padx=2, expand=True, fill="x")

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
        panel['distance_entry'].insert(0, "100")
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

