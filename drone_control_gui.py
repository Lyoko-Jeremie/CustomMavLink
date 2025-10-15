"""
无人机控制GUI程序 - 简单调试界面
使用tkinter创建图形界面，每个按钮对应一个控制指令
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import logging
from datetime import datetime
from airplane_manager_owl02 import create_manager, create_manager_with_serial

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DroneControlGUI:
    """无人机控制GUI类"""

    def __init__(self, root):
        self.root = root
        self.root.title("无人机控制调试界面")
        self.root.geometry("900x820")

        self.manager = None
        self.drone = None
        self.drone_id = 1

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

        # 创建主容器 - 左右布局
        main_container = tk.Frame(self.root)
        main_container.pack(fill="both", expand=True, padx=10, pady=5)

        # 左侧面板
        left_panel = tk.Frame(main_container)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 右侧面板
        right_panel = tk.Frame(main_container)
        right_panel.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # ==================== 左侧内容 ====================
        # 初始化区域
        init_frame = ttk.LabelFrame(left_panel, text="初始化", padding=10)
        init_frame.pack(fill="x", pady=5)

        # COM口配置
        com_frame = tk.Frame(init_frame)
        com_frame.pack(fill="x", pady=5)

        tk.Label(com_frame, text="COM口:").pack(side="left", padx=5)
        self.com_port_entry = tk.Entry(com_frame, width=10)
        self.com_port_entry.insert(0, "COM3")
        self.com_port_entry.pack(side="left", padx=5)

        tk.Label(com_frame, text="波特率:").pack(side="left", padx=5)
        self.baudrate_entry = tk.Entry(com_frame, width=10)
        self.baudrate_entry.insert(0, "115200")
        self.baudrate_entry.pack(side="left", padx=5)

        # 无人机ID输入
        id_frame = tk.Frame(init_frame)
        id_frame.pack(fill="x", pady=5)
        tk.Label(id_frame, text="无人机ID:").pack(side="left", padx=5)
        self.id_entry = tk.Entry(id_frame, width=10)
        self.id_entry.insert(0, "1")
        self.id_entry.pack(side="left", padx=5)

        # 初始化按钮
        btn_init = tk.Button(
            init_frame,
            text="初始化管理器",
            command=self.init_manager,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"),
            width=20,
            height=2
        )
        btn_init.pack(pady=5)

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

        # ==================== 右侧内容 ====================
        # Goto控制区域
        goto_frame = ttk.LabelFrame(right_panel, text="定点飞行", padding=10)
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

        # 日志输出区域
        log_frame = ttk.LabelFrame(right_panel, text="日志输出", padding=10)
        log_frame.pack(fill="both", expand=True, pady=5)

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

        # 底部状态栏
        self.status_label = tk.Label(
            self.root,
            text="状态: 未初始化",
            bg="#F0F0F0",
            anchor="w",
            padx=10
        )
        self.status_label.pack(fill="x", side="bottom")

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
        def _init():
            self.log_message("正在初始化管理器...")
            com_port = self.com_port_entry.get()
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
            self.log_message("✓ 管理器初始化成功")
            self.update_status("管理器已初始化")

        self.run_in_thread(_init)

    def get_drone(self):
        """获取无人机对象"""
        def _get():
            if not self.manager:
                self.log_message("请先初始化管理器", "ERROR")
                messagebox.showwarning("警告", "请先初始化管理器")
                return

            try:
                self.drone_id = int(self.id_entry.get())
            except ValueError:
                self.log_message("无效的无人机ID", "ERROR")
                messagebox.showerror("错误", "请输入有效的无人机ID")
                return

            self.log_message(f"正在获取无人机 (ID={self.drone_id})...")
            self.drone = self.manager.get_airplane(self.drone_id)
            self.log_message(f"✓ 无人机对象获取成功 (ID={self.drone_id})")
            self.update_status(f"无人机已连接 (ID={self.drone_id})")

        self.run_in_thread(_get)

    def check_drone(self):
        """检查无人机是否已初始化"""
        if not self.drone:
            self.log_message("请先获取无人机对象", "ERROR")
            messagebox.showwarning("警告", "请先初始化管理器并获取无人机对象")
            return False
        return True

    def arm(self):
        """解锁无人机"""
        if not self.check_drone():
            return

        def _arm():
            self.log_message("正在解锁无人机...")
            self.drone.arm()
            self.log_message("✓ 解锁命令已发送")

        self.run_in_thread(_arm)

    def disarm(self):
        """上锁无人机"""
        if not self.check_drone():
            return

        def _disarm():
            self.log_message("正在上锁无人机...")
            self.drone.disarm()
            self.log_message("✓ 上锁命令已发送")

        self.run_in_thread(_disarm)

    def takeoff(self):
        """起飞"""
        if not self.check_drone():
            return

        try:
            height = int(self.takeoff_height.get())
        except ValueError:
            self.log_message("无效的起飞高度", "ERROR")
            messagebox.showerror("错误", "请输入有效的起飞高度(cm)")
            return

        def _takeoff():
            self.log_message(f"正在起飞到 {height}cm...")
            self.drone.takeoff(height)
            self.log_message(f"✓ 起飞命令已发送 (高度: {height}cm)")

        self.run_in_thread(_takeoff)

    def land(self):
        """降落"""
        if not self.check_drone():
            return

        def _land():
            self.log_message("正在降落...")
            self.drone.land()
            self.log_message("✓ 降落命令已发送")

        self.run_in_thread(_land)

    def forward(self):
        """前进"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _forward():
            self.log_message(f"前进 {distance}cm...")
            self.drone.forward(distance)
            self.log_message(f"✓ 前进命令已发送")

        self.run_in_thread(_forward)

    def back(self):
        """后退"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _back():
            self.log_message(f"后退 {distance}cm...")
            self.drone.back(distance)
            self.log_message(f"✓ 后退命令已发送")

        self.run_in_thread(_back)

    def left(self):
        """左移"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _left():
            self.log_message(f"左移 {distance}cm...")
            self.drone.left(distance)
            self.log_message(f"✓ 左移命令已发送")

        self.run_in_thread(_left)

    def right(self):
        """右移"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _right():
            self.log_message(f"右移 {distance}cm...")
            self.drone.right(distance)
            self.log_message(f"✓ 右移命令已发送")

        self.run_in_thread(_right)

    def up(self):
        """上升"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _up():
            self.log_message(f"上升 {distance}cm...")
            self.drone.up(distance)
            self.log_message(f"✓ 上升命令已发送")

        self.run_in_thread(_up)

    def down(self):
        """下降"""
        if not self.check_drone():
            return

        try:
            distance = int(self.move_distance.get())
        except ValueError:
            self.log_message("无效的移动距离", "ERROR")
            return

        def _down():
            self.log_message(f"下降 {distance}cm...")
            self.drone.down(distance)
            self.log_message(f"✓ 下降命令已发送")

        self.run_in_thread(_down)

    def goto(self):
        """飞往目标点"""
        if not self.check_drone():
            return

        try:
            x = int(self.goto_x.get())
            y = int(self.goto_y.get())
            z = int(self.goto_z.get())
        except ValueError:
            self.log_message("无效的坐标值", "ERROR")
            messagebox.showerror("错误", "请输入有效的坐标值(cm)")
            return

        def _goto():
            self.log_message(f"飞往目标点 ({x}, {y}, {z})...")
            self.drone.goto(x, y, z)
            self.log_message(f"✓ Goto命令已发送 (X:{x}, Y:{y}, Z:{z})")

        self.run_in_thread(_goto)

    def on_closing(self):
        """关闭窗口时的处理"""
        if self.manager:
            self.log_message("正在停止管理器...")
            try:
                self.manager.stop()
                self.log_message("✓ 管理器已停止")
            except Exception as e:
                self.log_message(f"停止管理器时出错: {e}", "ERROR")

        self.root.destroy()


def main():
    """主函数"""
    root = tk.Tk()
    app = DroneControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
