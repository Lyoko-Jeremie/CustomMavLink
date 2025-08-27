# 无人机管理系统

这是一个基于Python的无人机管理系统，参考TypeScript实现，提供了无人机对象管理和MavLink协议通信功能。

## 主要功能

### 1. 无人机对象类 (AirplaneOwl02)

- **状态管理**: 解锁状态、飞行模式、GPS位置、电池状态等
- **MavLink包缓存**: 缓存接收到的每种MavLink包的最后一个包
- **消息解析**: 支持心跳、GPS位置、电池状态、命令确认等消息解析
- **控制接口**: 提供解锁、起飞、降落、返航等控制命令

### 2. 无人机管理类 (AirplaneManagerOwl02)

- **设备管理**: 根据设备ID自动创建和管理无人机对象
- **串口通信**: 接收串口数据并解析自定义协议
- **消息分发**: 将MavLink消息分发给对应的无人机对象
- **心跳发送**: 每1Hz向所有无人机发送心跳包
- **多线程处理**: 异步处理数据接收和消息发送

## 文件结构

```
├── airplane_owl02.py              # 无人机对象类
├── airplane_manager_owl02.py      # 无人机管理类
├── airplane_control_example.py    # 实际使用示例
├── test_airplane_system.py        # 系统测试脚本
├── main.py                        # 基础协议处理
└── README.md                      # 说明文档
```

## 依赖安装

```bash
pip install -r requirements.txt
```

需要的依赖包：
- `pymavlink`: MavLink协议处理
- `pyserial`: 串口通信

## 使用方法

### 1. 基本使用

```python
from airplane_manager_owl02 import create_manager_with_serial, create_manager
import asyncio

# 创建带串口的管理器
manager = create_manager_with_serial('COM3', 115200)

# 或创建不带串口的管理器（用于测试）
manager = create_manager()

# 初始化管理器
manager.init()

# 获取或创建无人机对象
async def main():
    airplane = await manager.get_airplane(1)  # 设备ID为1的无人机
    
    # 发送控制命令
    await airplane.arm()              # 解锁
    await airplane.takeoff(10.0)      # 起飞到10米
    await airplane.return_to_launch() # 返航
    await airplane.land()             # 降落
    await airplane.disarm()           # 锁定

# 运行
asyncio.run(main())

# 停止管理器
manager.stop()
```

### 2. 运行示例程序

```bash
# 运行测试脚本
python test_airplane_system.py

# 运行控制示例
python airplane_control_example.py
```

### 3. 交互式控制

运行 `airplane_control_example.py` 并选择交互模式，可以通过命令行控制无人机：

```
可用命令:
  list - 列出所有无人机
  stats - 显示统计信息
  arm <id> - 解锁指定无人机
  disarm <id> - 锁定指定无人机
  takeoff <id> <altitude> - 起飞到指定高度
  land <id> - 降落
  rtl <id> - 返航
  quit - 退出
```

## 自定义协议格式

系统使用以下自定义协议格式封装MavLink数据：

```
帧头1  帧头2  设备ID    数据长度  MavLink数据  校验和  帧尾
0xAA   0xBB   1-16      0-58      ...         sum     0xCC
```

- **设备ID**: 1-16，用于区分不同的无人机
- **MavLink数据**: 封装的MavLink消息，最大58字节
- **校验和**: 对包体所有字节求和的低8位

## 无人机状态

### 飞行模式
- `FLY_MODE_HOLD`: 定高模式
- `FLY_MODE_POSITION`: 定点模式  
- `FLY_MODE_AUTO`: 自动模式

### 自动模式细分
- `FLY_MODE_AUTO_TAKEOFF`: 自动起飞
- `FLY_MODE_AUTO_FOLLOW`: 自动跟踪
- `FLY_MODE_AUTO_MISSION`: 自动任务
- `FLY_MODE_AUTO_RTL`: 自动返航
- `FLY_MODE_AUTO_LAND`: 自动降落

### 定点模式细分
- `FLY_MODE_STABLE_NORMAL`: 普通定点
- `FLY_MODE_STABLE_OBSTACLE_AVOIDANCE`: 定点避障

## 支持的MavLink消息

- `HEARTBEAT`: 心跳包
- `EXTENDED_SYS_STATE`: 扩展系统状态
- `AUTOPILOT_VERSION`: 自动驾驶仪版本
- `STATUSTEXT`: 状态文本
- `COMMAND_ACK`: 命令确认
- `GLOBAL_POSITION_INT`: GPS位置
- `BATTERY_STATUS`: 电池状态

## 控制命令

- `arm()`: 解锁无人机
- `disarm()`: 锁定无人机
- `takeoff(altitude)`: 起飞到指定高度
- `land()`: 降落
- `return_to_launch()`: 返航
- `send_heartbeat()`: 发送心跳
- `trigger_get_autopilot_version()`: 请求版本信息

## 状态查询

- `get_state()`: 获取完整状态
- `get_gps_pos()`: 获取GPS位置
- `get_attitude()`: 获取姿态信息
- `get_cached_packet(msg_id)`: 获取缓存的特定消息

## 注意事项

1. **串口配置**: 确保串口参数正确（波特率、端口号等）
2. **设备ID**: 设备ID范围为1-16，需要与实际硬件配置一致
3. **异步编程**: 所有控制命令都是异步的，需要使用`await`关键字
4. **错误处理**: 建议在实际使用中添加适当的错误处理
5. **资源管理**: 使用完毕后记得调用`manager.stop()`释放资源

## 扩展功能

可以根据需要扩展以下功能：

1. **更多MavLink消息支持**: 在`parse_table`中添加新的消息处理函数
2. **自定义控制命令**: 继承`AirplaneOwl02`类添加新的控制接口
3. **状态持久化**: 添加状态存储和恢复功能
4. **GUI界面**: 基于此系统开发图形化控制界面
5. **数据记录**: 添加飞行数据记录和分析功能

## 示例输出

```
2025-08-27 10:30:15 - AirplaneManagerOwl02 - INFO - Initializing AirplaneManagerOwl02
2025-08-27 10:30:15 - AirplaneManagerOwl02 - INFO - Created new airplane with ID: 1
2025-08-27 10:30:15 - AirplaneOwl02 - INFO - Sent heartbeat to device 1
2025-08-27 10:30:16 - AirplaneOwl02 - INFO - Sent arm command to device 1
2025-08-27 10:30:18 - AirplaneOwl02 - INFO - Sent takeoff command to device 1 at altitude 10.0
```
