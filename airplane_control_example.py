"""
无人机管理系统实际使用示例
"""
import asyncio
import time
import logging
import signal
import sys
from airplane_manager_owl02 import AirplaneManagerOwl02, create_manager_with_serial, create_manager
from airplane_owl02 import AirplaneOwl02

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class AirplaneControlSystem:
    """无人机控制系统"""
    
    def __init__(self, serial_port: str = None, baudrate: int = 115200):
        self.manager = None
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.is_running = False
        
    def start(self):
        """启动系统"""
        try:
            if self.serial_port:
                logger.info(f"Starting system with serial port: {self.serial_port}")
                self.manager = create_manager_with_serial(self.serial_port, self.baudrate)
            else:
                logger.info("Starting system without serial port (simulation mode)")
                self.manager = create_manager()
                
            self.manager.init()
            self.is_running = True
            logger.info("Airplane control system started successfully")
            
        except Exception as e:
            logger.error(f"Failed to start system: {e}")
            raise
            
    def stop(self):
        """停止系统"""
        if self.manager:
            self.manager.stop()
        self.is_running = False
        logger.info("Airplane control system stopped")
        
    async def monitor_airplanes(self):
        """监控所有无人机状态"""
        while self.is_running:
            try:
                stats = self.manager.get_statistics()
                
                if stats['airplane_count'] > 0:
                    logger.info(f"Monitoring {stats['airplane_count']} airplanes:")
                    
                    for device_id, airplane_stats in stats['airplanes'].items():
                        logger.info(f"  Airplane {device_id}: "
                                  f"Armed={airplane_stats['is_armed']}, "
                                  f"Mode={airplane_stats['fly_mode']}, "
                                  f"Landed={airplane_stats['is_landed']}, "
                                  f"GPS=({airplane_stats['gps_position']['lat']:.6f}, "
                                  f"{airplane_stats['gps_position']['lon']:.6f}, "
                                  f"{airplane_stats['gps_position']['alt']:.2f}m)")
                else:
                    logger.info("No airplanes connected")
                    
                await asyncio.sleep(5)  # 每5秒监控一次
                
            except Exception as e:
                logger.error(f"Error in monitoring: {e}")
                await asyncio.sleep(1)
                
    async def control_airplane_example(self, device_id: int):
        """控制无人机示例"""
        try:
            # 获取无人机对象
            airplane = await self.manager.get_airplane(device_id)
            logger.info(f"Controlling airplane {device_id}")
            
            # 检查初始状态
            state = airplane.get_state()
            logger.info(f"Initial state: armed={state.is_armed}, mode={state.fly_mode}")
            
            # 控制序列示例
            logger.info("Step 1: Requesting autopilot version...")
            await airplane.trigger_get_autopilot_version()
            await asyncio.sleep(1)
            
            logger.info("Step 2: Arming the airplane...")
            await airplane.arm()
            await asyncio.sleep(2)
            
            logger.info("Step 3: Takeoff to 10 meters...")
            await airplane.takeoff(10.0)
            await asyncio.sleep(5)
            
            logger.info("Step 4: Hold position for 10 seconds...")
            await asyncio.sleep(10)
            
            logger.info("Step 5: Return to launch...")
            await airplane.return_to_launch()
            await asyncio.sleep(5)
            
            logger.info("Step 6: Landing...")
            await airplane.land()
            await asyncio.sleep(3)
            
            logger.info("Step 7: Disarming...")
            await airplane.disarm()
            
            logger.info(f"Control sequence completed for airplane {device_id}")
            
        except Exception as e:
            logger.error(f"Error controlling airplane {device_id}: {e}")
            
    async def simulate_multiple_airplanes(self):
        """模拟多个无人机操作"""
        tasks = []
        
        # 创建多个无人机的控制任务
        for device_id in [1, 2, 3]:
            task = asyncio.create_task(self.control_airplane_example(device_id))
            tasks.append(task)
            # 错开启动时间
            await asyncio.sleep(2)
            
        # 等待所有任务完成
        await asyncio.gather(*tasks, return_exceptions=True)
        
    def run_interactive_mode(self):
        """交互式模式"""
        logger.info("进入交互式模式，输入命令控制无人机")
        logger.info("可用命令:")
        logger.info("  list - 列出所有无人机")
        logger.info("  stats - 显示统计信息")
        logger.info("  arm <id> - 解锁指定无人机")
        logger.info("  disarm <id> - 锁定指定无人机")
        logger.info("  takeoff <id> <altitude> - 起飞到指定高度")
        logger.info("  land <id> - 降落")
        logger.info("  rtl <id> - 返航")
        logger.info("  quit - 退出")
        
        while self.is_running:
            try:
                command = input("\n请输入命令: ").strip().split()
                if not command:
                    continue
                    
                cmd = command[0].lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'list':
                    airplanes = self.manager.get_airplane_list()
                    if airplanes:
                        logger.info(f"已连接的无人机: {list(airplanes.keys())}")
                    else:
                        logger.info("没有连接的无人机")
                        
                elif cmd == 'stats':
                    stats = self.manager.get_statistics()
                    logger.info(f"统计信息: {stats}")
                    
                elif cmd == 'arm' and len(command) > 1:
                    device_id = int(command[1])
                    asyncio.run(self._arm_airplane(device_id))
                    
                elif cmd == 'disarm' and len(command) > 1:
                    device_id = int(command[1])
                    asyncio.run(self._disarm_airplane(device_id))
                    
                elif cmd == 'takeoff' and len(command) > 2:
                    device_id = int(command[1])
                    altitude = float(command[2])
                    asyncio.run(self._takeoff_airplane(device_id, altitude))
                    
                elif cmd == 'land' and len(command) > 1:
                    device_id = int(command[1])
                    asyncio.run(self._land_airplane(device_id))
                    
                elif cmd == 'rtl' and len(command) > 1:
                    device_id = int(command[1])
                    asyncio.run(self._rtl_airplane(device_id))
                    
                else:
                    logger.warning("未知命令或参数不足")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"命令执行错误: {e}")
                
    async def _arm_airplane(self, device_id: int):
        airplane = await self.manager.get_airplane(device_id)
        await airplane.arm()
        logger.info(f"已发送解锁命令给无人机 {device_id}")
        
    async def _disarm_airplane(self, device_id: int):
        airplane = await self.manager.get_airplane(device_id)
        await airplane.disarm()
        logger.info(f"已发送锁定命令给无人机 {device_id}")
        
    async def _takeoff_airplane(self, device_id: int, altitude: float):
        airplane = await self.manager.get_airplane(device_id)
        await airplane.takeoff(altitude)
        logger.info(f"已发送起飞命令给无人机 {device_id}，目标高度 {altitude} 米")
        
    async def _land_airplane(self, device_id: int):
        airplane = await self.manager.get_airplane(device_id)
        await airplane.land()
        logger.info(f"已发送降落命令给无人机 {device_id}")
        
    async def _rtl_airplane(self, device_id: int):
        airplane = await self.manager.get_airplane(device_id)
        await airplane.return_to_launch()
        logger.info(f"已发送返航命令给无人机 {device_id}")

def signal_handler(signum, frame):
    """信号处理器"""
    logger.info("接收到退出信号，正在关闭系统...")
    sys.exit(0)

def main():
    """主函数"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 配置参数
    SERIAL_PORT = None  # 设置为实际的串口，如 'COM3' 或 '/dev/ttyUSB0'
    BAUDRATE = 115200
    
    # 创建控制系统
    control_system = AirplaneControlSystem(SERIAL_PORT, BAUDRATE)
    
    try:
        # 启动系统
        control_system.start()
        
        # 选择运行模式
        print("选择运行模式:")
        print("1. 监控模式 - 监控所有连接的无人机")
        print("2. 模拟模式 - 模拟多个无人机操作")
        print("3. 交互模式 - 手动控制无人机")
        
        choice = input("请选择模式 (1-3): ").strip()
        
        if choice == '1':
            logger.info("启动监控模式")
            asyncio.run(control_system.monitor_airplanes())
            
        elif choice == '2':
            logger.info("启动模拟模式")
            asyncio.run(control_system.simulate_multiple_airplanes())
            
        elif choice == '3':
            logger.info("启动交互模式")
            control_system.run_interactive_mode()
            
        else:
            logger.error("无效选择")
            
    except KeyboardInterrupt:
        logger.info("用户中断程序")
    except Exception as e:
        logger.error(f"程序运行错误: {e}")
    finally:
        control_system.stop()
        logger.info("程序已退出")

if __name__ == '__main__':
    main()
