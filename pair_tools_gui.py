"""
一个GUI界面，用于无人机配对的工具。

功能及工作流程：
打开一个或多个无人机串口，
从无人机上读取无人机配对包，
记录当前读取到的无人机配对包，并显示其Hex数据，
在GUI中显示已经记录的所有无人机配对包Hex数据，

打开一个或多个地面板串口，
让用户选择一个无人机配对包并写入到地面板的指定通道上并显示是否写入成功。

在GUI中显示当前地面板上的0~15个通道的无人机配对包Hex数据。

需要区分连接到无人机的多个串口和连接到地面板的多个串口。

"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
from owl2.pair_manager import PairManager, AirplaneId


class PairToolsGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("无人机配对工具")
        self.root.geometry("1200x800")

        # 配对管理器
        self.pair_manager = PairManager()

        # 串口连接字典
        self.drone_ports = {}  # {port_name: serial.Serial}
        self.board_ports = {}  # {port_name: serial.Serial}

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
        self.drone_baud_combo.set('115200')
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

        # 已连接串口列表
        list_frame = ttk.LabelFrame(parent, text="已连接串口", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.board_ports_listbox = tk.Listbox(list_frame, height=4)
        self.board_ports_listbox.pack(fill=tk.BOTH, expand=True)

        # 配对操作区域
        pair_frame = ttk.LabelFrame(parent, text="配对操作", padding=10)
        pair_frame.pack(fill=tk.X, padx=5, pady=5)

        # 通道选择
        channel_frame = ttk.Frame(pair_frame)
        channel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(channel_frame, text="目标通道:").pack(side=tk.LEFT, padx=5)
        self.channel_spinbox = ttk.Spinbox(channel_frame, from_=0, to=15, width=10)
        self.channel_spinbox.set('0')
        self.channel_spinbox.pack(side=tk.LEFT, padx=5)

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
        """连接地面板串口"""
        port_name = self.board_port_combo.get()
        if not port_name:
            messagebox.showwarning("警告", "请选择串口")
            return

        if port_name in self.board_ports:
            messagebox.showinfo("提示", f"串口 {port_name} 已连接")
            return

        try:
            baud_rate = int(self.board_baud_combo.get())
            ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.board_ports[port_name] = ser
            self.board_ports_listbox.insert(tk.END, f"{port_name} ({baud_rate})")
            messagebox.showinfo("成功", f"成功连接串口 {port_name}")
        except Exception as e:
            messagebox.showerror("错误", f"连接串口失败: {str(e)}")

    def _disconnect_board_port(self):
        """断开地面板串口"""
        selection = self.board_ports_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请选择要断开的串口")
            return

        index = selection[0]
        port_info = self.board_ports_listbox.get(index)
        port_name = port_info.split()[0]

        if port_name in self.board_ports:
            self.board_ports[port_name].close()
            del self.board_ports[port_name]
            self.board_ports_listbox.delete(index)
            messagebox.showinfo("成功", f"已断开串口 {port_name}")

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

        # 在新线程中读取，避免界面冻结
        def read_thread():
            try:
                serial_port = self.drone_ports[port_name]
                airplane_id = self.pair_manager.get_airplane_id_from_serial(serial_port, timeout=3.0)

                # 添加到列表
                self.airplane_ids.append(airplane_id)

                # 更新界面
                self.root.after(0, self._update_drone_id_list)
                self.root.after(0, lambda: messagebox.showinfo("成功",
                    f"成功读取无人机ID\nMTX地址: {airplane_id.addr_hex_str}"))

            except TimeoutError as e:
                self.root.after(0, lambda: messagebox.showerror("超时", str(e)))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取失败: {str(e)}"))

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
        board_selection = self.board_ports_listbox.curselection()
        if not board_selection:
            messagebox.showwarning("警告", "请先选择一个地面板串口")
            return

        # 获取选中的无人机ID
        item = selection[0]
        values = self.drone_id_tree.item(item)['values']
        index = int(values[0]) - 1
        airplane_id = self.airplane_ids[index]

        # 获取目标通道
        try:
            channel = int(self.channel_spinbox.get())
            if not (0 <= channel <= 15):
                messagebox.showerror("错误", "通道号必须在0-15之间")
                return
        except ValueError:
            messagebox.showerror("错误", "通道号必须是数字")
            return

        # 获取地面板串口
        board_index = board_selection[0]
        port_info = self.board_ports_listbox.get(board_index)
        port_name = port_info.split()[0]

        if port_name not in self.board_ports:
            messagebox.showerror("错误", "地面板串口未连接")
            return

        # 在新线程中写入
        def write_thread():
            try:
                serial_port = self.board_ports[port_name]
                success = self.pair_manager.set_airplane_id_to_channel(serial_port, channel, airplane_id, timeout=3.0)

                if success:
                    # 更新通道信息
                    self.board_channels[channel] = airplane_id
                    self.root.after(0, self._update_channels_list)
                    self.root.after(0, lambda: messagebox.showinfo("成功",
                        f"成功写入配对\n通道: {channel}\nMTX地址: {airplane_id.addr_hex_str}"))
                else:
                    self.root.after(0, lambda: messagebox.showerror("失败", "配对写入失败"))

            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"写入失败: {str(e)}"))

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
                addr = self.board_channels[channel].addr_hex_str
            else:
                addr = "未配对"
            self.channels_tree.insert('', tk.END, values=(channel, addr))

    def run(self):
        """运行GUI"""
        self.root.mainloop()

        # 关闭所有串口
        for port in self.drone_ports.values():
            port.close()
        for port in self.board_ports.values():
            port.close()


def main():
    root = tk.Tk()
    app = PairToolsGUI(root)
    app.run()


if __name__ == "__main__":
    main()
