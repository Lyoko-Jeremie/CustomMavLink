"""
一个GUI界面，用于无人机配对的工具。

功能及工作流程：
打开一个或多个无人机串口，
从无人机上读取无人机配对包，
记录当前读取到的无人机配对包，并显示其Hex数据，
在GUI中显示已经记录的所有无人机配对包Hex数据，

打开一个地面板串口，
让用户选择一个无人机配对包并写入到地面板的指定通道上并显示是否写入成功。

在GUI中显示当前地面板上的0~15个通道的无人机配对包Hex数据。

需要区分连接到无人机的多个串口，但只有一个连接到地面板的串口。

"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
from owl2.pair_manager import PairManager


class PairToolsGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("无人机配对工具")
        self.root.geometry("1200x900")

        # 配对管理器
        self.pair_manager = PairManager()

        # 串口连接字典
        self.drone_ports = {}  # {port_name: serial.Serial}
        self.board_port = None  # 只保存一个地面板串口连接
        self.board_port_name = None  # 当前连接的地面板串口名称

        # 已读取的无人机ID列表
        self.airplane_ids = []  # List[AirplaneId]

        # 地面板通道配对信息
        self.board_channels = {}  # {channel: AirplaneId}

        self._create_ui()

    def _create_ui(self):
        """创建用户界面"""
        # 设置样式
        style = ttk.Style()
        style.theme_use('clam')

        # 创建主容器
        main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧面板：无人机管理
        left_frame = ttk.Frame(main_container)
        main_container.add(left_frame, weight=1)
        self._create_drone_panel(left_frame)

        # 右侧面板：地面板管理
        right_frame = ttk.Frame(main_container)
        main_container.add(right_frame, weight=1)
        self._create_board_panel(right_frame)

    def _create_drone_panel(self, parent):
        """创建无人机管理面板"""
        # 标题
        title_frame = ttk.Frame(parent)
        title_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(title_frame, text="无人机管理", font=('Arial', 14, 'bold')).pack(side=tk.LEFT)

        # 串口连接区域
        conn_frame = ttk.LabelFrame(parent, text="串口连接", padding=10)
        conn_frame.pack(fill=tk.X, padx=5, pady=5)

        # 串口选择
        port_select_frame = ttk.Frame(conn_frame)
        port_select_frame.pack(fill=tk.X, pady=2)
        ttk.Label(port_select_frame, text="串口:").pack(side=tk.LEFT, padx=5)
        self.drone_port_combo = ttk.Combobox(port_select_frame, width=15, state='readonly')
        self.drone_port_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(port_select_frame, text="刷新", command=self._refresh_drone_ports).pack(side=tk.LEFT, padx=2)

        # 波特率选择
        baud_frame = ttk.Frame(conn_frame)
        baud_frame.pack(fill=tk.X, pady=2)
        ttk.Label(baud_frame, text="波特率:").pack(side=tk.LEFT, padx=5)
        self.drone_baud_combo = ttk.Combobox(baud_frame, width=15, state='readonly',
                                             values=['9600', '19200', '38400', '57600', '115200'])
        self.drone_baud_combo.set('57600')
        self.drone_baud_combo.pack(side=tk.LEFT, padx=5)

        # 连接/断开按钮
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="连接", command=self._connect_drone_port).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="断开", command=self._disconnect_drone_port).pack(side=tk.LEFT, padx=2)

        # 已连接串口列表
        list_frame = ttk.LabelFrame(parent, text="已连接串口", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.drone_ports_listbox = tk.Listbox(list_frame, height=4)
        self.drone_ports_listbox.pack(fill=tk.BOTH, expand=True)

        # 读取ID按钮
        ttk.Button(list_frame, text="读取无人机ID", command=self._read_drone_id).pack(fill=tk.X, pady=5)

        # 已读取的无人机ID列表
        id_frame = ttk.LabelFrame(parent, text="已读取的无人机ID", padding=10)
        id_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 创建表格
        columns = ("序号", "MTX地址")
        self.drone_id_tree = ttk.Treeview(id_frame, columns=columns, show='headings', height=10)
        self.drone_id_tree.heading("序号", text="序号")
        self.drone_id_tree.heading("MTX地址", text="MTX地址 (Hex)")
        self.drone_id_tree.column("序号", width=50)
        self.drone_id_tree.column("MTX地址", width=200)

        # 滚动条
        scrollbar = ttk.Scrollbar(id_frame, orient=tk.VERTICAL, command=self.drone_id_tree.yview)
        self.drone_id_tree.configure(yscrollcommand=scrollbar.set)

        self.drone_id_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 状态显示区域
        status_frame = ttk.LabelFrame(parent, text="状态", padding=10)
        status_frame.pack(fill=tk.X, padx=5, pady=5)

        # 无人机读取状态
        self.drone_status_label = ttk.Label(status_frame, text="就绪", foreground='blue', font=('Arial', 12))
        self.drone_status_label.pack(side=tk.LEFT, padx=5)

        # 初始化端口列表
        self._refresh_drone_ports()

    def _create_board_panel(self, parent):
        """创建地面板管理面板"""
        # 标题
        title_frame = ttk.Frame(parent)
        title_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(title_frame, text="地面板管理", font=('Arial', 14, 'bold')).pack(side=tk.LEFT)

        # 串口连接区域
        conn_frame = ttk.LabelFrame(parent, text="串口连接", padding=10)
        conn_frame.pack(fill=tk.X, padx=5, pady=5)

        # 串口选择
        port_select_frame = ttk.Frame(conn_frame)
        port_select_frame.pack(fill=tk.X, pady=2)
        ttk.Label(port_select_frame, text="串口:").pack(side=tk.LEFT, padx=5)
        self.board_port_combo = ttk.Combobox(port_select_frame, width=15, state='readonly')
        self.board_port_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(port_select_frame, text="刷新", command=self._refresh_board_ports).pack(side=tk.LEFT, padx=2)

        # 波特率选择
        baud_frame = ttk.Frame(conn_frame)
        baud_frame.pack(fill=tk.X, pady=2)
        ttk.Label(baud_frame, text="波特率:").pack(side=tk.LEFT, padx=5)
        self.board_baud_combo = ttk.Combobox(baud_frame, width=15, state='readonly',
                                             values=['9600', '19200', '38400', '57600', '115200'])
        self.board_baud_combo.set('115200')
        self.board_baud_combo.pack(side=tk.LEFT, padx=5)

        # 连接/断开按钮
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="连接", command=self._connect_board_port).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="断开", command=self._disconnect_board_port).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="读取通道信息", command=self._refresh_board_channels).pack(side=tk.LEFT, padx=2)

        # 配对操作区域
        pair_frame = ttk.LabelFrame(parent, text="配对操作", padding=10)
        pair_frame.pack(fill=tk.X, padx=5, pady=5)

        # 通道选择
        channel_frame = ttk.Frame(pair_frame)
        channel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(channel_frame, text="目标通道:").pack(side=tk.LEFT, padx=5)
        # 使用Combobox替代Spinbox，设置为只读，确保只能选择0-15
        self.channel_combo = ttk.Combobox(channel_frame, width=10, state='readonly',
                                          values=[str(i) for i in range(16)])  # 0-15共16个通道，转为字符串
        self.channel_combo.set('0')
        self.channel_combo.pack(side=tk.LEFT, padx=5)
        ttk.Label(channel_frame, text="(可选范围: 0-15)", font=('Arial', 9)).pack(side=tk.LEFT, padx=5)

        # 配对按钮
        ttk.Button(pair_frame, text="写入配对到地面板", command=self._write_pair_to_board).pack(fill=tk.X, pady=5)

        # 地面板通道信息
        channels_frame = ttk.LabelFrame(parent, text="地面板通道配对信息 (0-15)", padding=10)
        channels_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 创建通道表格
        columns = ("通道", "MTX地址")
        self.channels_tree = ttk.Treeview(channels_frame, columns=columns, show='headings', height=10)
        self.channels_tree.heading("通道", text="通道")
        self.channels_tree.heading("MTX地址", text="MTX地址 (Hex)")
        self.channels_tree.column("通道", width=80)
        self.channels_tree.column("MTX地址", width=200)

        # 滚动条
        scrollbar = ttk.Scrollbar(channels_frame, orient=tk.VERTICAL, command=self.channels_tree.yview)
        self.channels_tree.configure(yscrollcommand=scrollbar.set)

        self.channels_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 状态显示区域
        status_frame = ttk.LabelFrame(parent, text="状态", padding=10)
        status_frame.pack(fill=tk.X, padx=5, pady=5)

        # 地面板连接状态
        self.board_status_label = ttk.Label(status_frame, text="未连接", foreground='red', font=('Arial', 12))
        self.board_status_label.pack(side=tk.LEFT, padx=5)

        # 初始化端口列表
        self._refresh_board_ports()

    def _refresh_drone_ports(self):
        """刷新无人机串口列表"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.drone_port_combo['values'] = ports
        if ports and not self.drone_port_combo.get():
            self.drone_port_combo.current(0)

    def _refresh_board_ports(self):
        """刷新地面板串口列表"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.board_port_combo['values'] = ports
        if ports and not self.board_port_combo.get():
            self.board_port_combo.current(0)

    def _connect_drone_port(self):
        """连接无人机串口"""
        port_name = self.drone_port_combo.get()
        if not port_name:
            messagebox.showwarning("警告", "请选择串口")
            return

        if port_name in self.drone_ports:
            messagebox.showinfo("提示", f"串口 {port_name} 已连接")
            return

        try:
            baud_rate = int(self.drone_baud_combo.get())
            ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.drone_ports[port_name] = ser
            self.drone_ports_listbox.insert(tk.END, f"{port_name} ({baud_rate})")
            messagebox.showinfo("成功", f"成功连接串口 {port_name}")
        except Exception as e:
            messagebox.showerror("错误", f"连接串口失败: {str(e)}")

    def _disconnect_drone_port(self):
        """断开无人机串口"""
        selection = self.drone_ports_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请选择要断开的串口")
            return

        index = selection[0]
        port_info = self.drone_ports_listbox.get(index)
        port_name = port_info.split()[0]

        if port_name in self.drone_ports:
            self.drone_ports[port_name].close()
            del self.drone_ports[port_name]
            self.drone_ports_listbox.delete(index)
            messagebox.showinfo("成功", f"已断开串口 {port_name}")

    def _connect_board_port(self):
        """连接地面板串口（仅支持一个连接）"""
        port_name = self.board_port_combo.get()
        if not port_name:
            messagebox.showwarning("警告", "请选择串口")
            return

        # 如果已经连接了相同的串口，提示用户
        if self.board_port and self.board_port_name == port_name:
            messagebox.showinfo("提示", f"串口 {port_name} 已连接")
            return

        # 如果已经连接了其他串口，先断开旧连接
        if self.board_port:
            old_port_name = self.board_port_name
            self._disconnect_board_port(silent=True)
            messagebox.showinfo("提示", f"已自动断开旧地面板 {old_port_name}，准备连接新地面板")

        try:
            baud_rate = int(self.board_baud_combo.get())
            ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.board_port = ser
            self.board_port_name = port_name

            # 清除旧的通道配对信息
            self.board_channels.clear()
            # 不立即更新通道列表，等待读取完成后再更新
            self._update_board_status()

            messagebox.showinfo("成功", f"成功连接地面板 {port_name}\n正在读取通道配对信息...")

            # 延迟后自动刷新通道列表
            self.root.after(500, self._refresh_board_channels)

        except Exception as e:
            messagebox.showerror("错误", f"连接串口失败: {str(e)}")
            self._update_board_status()

    def _disconnect_board_port(self, silent=False):
        """断开地面板串口"""
        if self.board_port:
            self.board_port.close()
            self.board_port = None
            self.board_port_name = None
            self.channels_tree.delete(*self.channels_tree.get_children())
            if not silent:
                messagebox.showinfo("成功", "已断开地面板串口")

    def _read_drone_id(self):
        """从选中的无人机串口读取ID"""
        selection = self.drone_ports_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请先选择一个无人机串口")
            return

        index = selection[0]
        port_info = self.drone_ports_listbox.get(index)
        port_name = port_info.split()[0]

        if port_name not in self.drone_ports:
            messagebox.showerror("错误", "串口未连接")
            return

        # 显示"正在读取"提示
        self._update_drone_status_message(f"正在从 {port_name} 读取无人机ID...", reading=True)

        # 在新线程中读取，避免界面冻结
        def read_thread():
            try:
                serial_port = self.drone_ports[port_name]
                airplane_id = self.pair_manager.get_airplane_id_from_serial(serial_port, timeout=5.0)

                # 添加到列表
                self.airplane_ids.append(airplane_id)

                # 更新界面
                self.root.after(0, self._update_drone_id_list)

                # 显示成功状态
                self.root.after(0, lambda: self._update_drone_status_message(
                    f"成功读取无人机ID: {airplane_id.addr_hex_str}", reading=False))

                self.root.after(0, lambda: messagebox.showinfo(
                    "成功",
                    f"成功读取无人机ID\nMTX地址: {airplane_id.addr_hex_str}"))

            except TimeoutError as e:
                self.root.after(0, lambda: self._update_drone_status_message("读取超时", error=True))
                self.root.after(0, lambda err=e: messagebox.showerror("超时", str(err)))
            except Exception as e:
                self.root.after(0, lambda: self._update_drone_status_message(f"读取失败", error=True))
                self.root.after(0, lambda err=e: messagebox.showerror("错误", f"读取失败: {str(err)}"))

        thread = threading.Thread(target=read_thread, daemon=True)
        thread.start()

    def _update_drone_id_list(self):
        """更新无人机ID列表显示"""
        # 清空现有项
        for item in self.drone_id_tree.get_children():
            self.drone_id_tree.delete(item)

        # 添加所有ID
        for idx, airplane_id in enumerate(self.airplane_ids, 1):
            self.drone_id_tree.insert('', tk.END, values=(idx, airplane_id.addr_hex_str))

    def _write_pair_to_board(self):
        """将选中的无人机ID写入地面板指定通道"""
        # 检查是否选择了无人机ID
        selection = self.drone_id_tree.selection()
        if not selection:
            messagebox.showwarning("警告", "请先选择一个无人机ID")
            return

        # 检查是否选择了地面板串口
        if not self.board_port:
            messagebox.showwarning("警告", "请先连接一个地面板串口")
            return

        # 获取选中的无人机ID
        item = selection[0]
        values = self.drone_id_tree.item(item)['values']
        index = int(values[0]) - 1
        airplane_id = self.airplane_ids[index]

        # 获取目标通道 - 从Combobox获取，确保值在0-15范围内
        try:
            channel_str = self.channel_combo.get()
            if not channel_str:
                messagebox.showerror("错误", "请选择目标通道")
                return

            channel = int(channel_str)
            # 双重验证：确保通道号在0-15之间
            if not (0 <= channel <= 15):
                messagebox.showerror("错误", f"通道号必须在0-15之间，当前值: {channel}")
                return
        except (ValueError, TypeError):
            messagebox.showerror("错误", "通道号格式错误，请选择0-15之间的数字")
            return

        # 获取地面板串口
        serial_port = self.board_port

        # 在新线程中写入
        def write_thread():
            try:
                success = self.pair_manager.set_airplane_id_to_channel(serial_port, channel, airplane_id, timeout=3.0)

                if success:
                    # 更新通道信息
                    self.board_channels[channel] = airplane_id
                    self.root.after(0, self._update_channels_list)
                    self.root.after(0, lambda: messagebox.showinfo("成功",
                                                                   f"成功写入配对\n通道: {channel}\nMTX地址: {airplane_id.addr_hex_str}"))
                else:
                    self.root.after(0, lambda: messagebox.showerror("失败", "配对写入失败"))

            except Exception as err:
                self.root.after(0, lambda e=err: messagebox.showerror("错误", f"写入失败: {str(e)}"))

        thread = threading.Thread(target=write_thread, daemon=True)
        thread.start()

    def _update_channels_list(self):
        """更新地面板通道列表显示"""
        # 清空现有项
        for item in self.channels_tree.get_children():
            self.channels_tree.delete(item)

        # 添加所有通道
        for channel in range(16):
            if channel in self.board_channels:
                airplane_id = self.board_channels[channel]
                if airplane_id is None:
                    addr = "未配对"
                else:
                    addr = airplane_id.addr_hex_str
            else:
                addr = "读取失败"
            self.channels_tree.insert('', tk.END, values=(channel, addr))

    def _update_board_status(self):
        """更新地面板连接状态显示"""
        if self.board_port and self.board_port_name:
            status_text = f"已连接: {self.board_port_name}"
            self.board_status_label.config(text=status_text, foreground='green')
        else:
            self.board_status_label.config(text="未连接", foreground='red')

    def _refresh_board_channels(self):
        """从地面板读取所有通道的配对信息并刷新显示"""
        if not self.board_port:
            return

        # 显示"正在读取"提示
        self.root.after(0, lambda: self._update_status_message("正在读取通道配对信息...", error=False))

        # 在新线程中读取，避免界面冻结
        def read_thread():
            try:
                # 从地面板读取所有通道的配对信息
                channels_data = self.pair_manager.get_all_channel_id_from_board(
                    self.board_port,
                    timeout=2.0
                )

                # 更新内部数据
                # channels_data中:
                # - 键存在且值为None: 该通道未配对
                # - 键存在且值为AirplaneId: 该通道已配对
                # - 键不存在: 该通道读取失败

                # 直接使用返回的字典替换当前数据
                self.board_channels = channels_data

                # 更新界面显示
                self.root.after(0, self._update_channels_list)

                # 统计并显示读取结果
                paired_count = sum(1 for v in channels_data.values() if v is not None)
                unpaired_count = sum(1 for v in channels_data.values() if v is None)
                failed_count = 16 - len(channels_data)

                status_msg = f"已读取 {len(channels_data)}/16 个通道 (已配对:{paired_count}, 未配对:{unpaired_count}, 读取失败:{failed_count})"
                self.root.after(0, lambda status_msg_s=status_msg: self._update_status_message(status_msg_s))

            except Exception as err:
                self.root.after(0, lambda e=err: self._update_status_message(
                    f"读取通道信息失败: {str(e)}", error=True
                ))

        thread = threading.Thread(target=read_thread, daemon=True)
        thread.start()

    def _update_status_message(self, message, error=False):
        """更新状态消息（临时显示在状态标签上）"""
        if self.board_port and self.board_port_name:
            if error:
                self.board_status_label.config(
                    text=f"已连接: {self.board_port_name} | {message}",
                    foreground='orange'
                )
            else:
                self.board_status_label.config(
                    text=f"已连接: {self.board_port_name} | {message}",
                    foreground='green'
                )

            # 2秒后恢复正常状态显示
            self.root.after(2000, self._update_board_status)

    def _update_drone_status_message(self, message, reading=False, error=False):
        """更新无人机状态消息（临时显示在状态标签上）"""
        if error:
            self.drone_status_label.config(text=message, foreground='red')
            # 2秒后恢复就绪状态
            self.root.after(2000, lambda: self.drone_status_label.config(text="就绪", foreground='blue'))
        elif reading:
            self.drone_status_label.config(text=message, foreground='orange')
        else:
            self.drone_status_label.config(text=message, foreground='green')
            # 2秒后恢复就绪状态
            self.root.after(2000, lambda: self.drone_status_label.config(text="就绪", foreground='blue'))
    def run(self):
        """运行GUI"""
        self.root.mainloop()

        # 关闭所有串口
        for port in self.drone_ports.values():
            port.close()
        if self.board_port:
            self.board_port.close()


def main():
    root = tk.Tk()
    app = PairToolsGUI(root)
    app.run()


if __name__ == "__main__":
    main()
