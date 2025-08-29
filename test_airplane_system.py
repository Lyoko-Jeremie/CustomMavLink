"""
测试无人机管理系统
演示如何使用AirplaneOwl02和AirplaneManagerOwl02类
"""
import asyncio
import time
import logging
from airplane_manager_owl02 import AirplaneManagerOwl02, create_manager_with_serial, create_manager
from airplane_owl02 import AirplaneOwl02

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_basic_functionality():
    """测试基本功能"""
    logger.info("=== 测试基本功能 ===")
    
    # 创建不带串口的管理器（用于测试）
    manager = create_manager()
    manager.init()
    
    # 模拟添加几个无人机
    def add_test_airplanes():
        airplane1 = manager.get_airplane(1)
        airplane2 = manager.get_airplane(2)
        airplane3 = manager.get_airplane(5)
        
        logger.info("Added test airplanes")
        return airplane1, airplane2, airplane3
    
    # 运行测试
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        airplane1, airplane2, airplane3 = add_test_airplanes()
        
        # 检查状态
        logger.info(f"Airplane 1 state: armed={airplane1.state.is_armed}, mode={airplane1.state.fly_mode}")
        logger.info(f"Airplane 2 state: armed={airplane2.state.is_armed}, mode={airplane2.state.fly_mode}")
        logger.info(f"Airplane 3 state: armed={airplane3.state.is_armed}, mode={airplane3.state.fly_mode}")
        
        # 获取统计信息
        stats = manager.get_statistics()
        logger.info(f"Manager statistics: {stats}")
        
        # 测试控制指令（模拟）
        def test_commands():
            logger.info("Testing control commands...")
            airplane1.arm()
            airplane1.takeoff(10.0)
            airplane2.arm()
            airplane3.return_to_launch()
            
        test_commands()
        
        logger.info("Basic functionality test completed")
        
    except Exception as e:
        logger.error(f"Error in basic functionality test: {e}")
    finally:
        manager.stop()
        loop.close()

def test_with_serial_simulation():
    """测试串口模拟"""
    logger.info("=== 测试串口模拟 ===")
    
    # 注意：这里需要实际的串口设备，或者使用虚拟串口进行测试
    # 如果没有实际设备，这部分测试会失败
    
    try:
        # 尝试创建带串口的管理器
        # 你需要根据实际情况修改端口名
        # manager = create_manager_with_serial('COM3', 115200)
        
        # 如果没有实际串口，创建不带串口的管理器
        manager = create_manager()
        manager.init()
        
        logger.info("Manager with serial simulation created")
        
        # 运行一段时间让心跳工作
        logger.info("Running for 5 seconds to test heartbeat...")
        start_time = time.time()
        
        while time.time() - start_time < 5:
            stats = manager.get_statistics()
            logger.info(f"Stats: {stats['airplane_count']} airplanes")
            time.sleep(1)
            
        manager.stop()
        logger.info("Serial simulation test completed")
        
    except Exception as e:
        logger.error(f"Error in serial simulation test: {e}")

def test_airplane_state_management():
    """测试无人机状态管理"""
    logger.info("=== 测试无人机状态管理 ===")
    
    manager = create_manager()
    manager.init()
    
    def test_state_changes():
        airplane = manager.get_airplane(1)
        
        # 测试状态获取
        initial_state = airplane.get_state()
        logger.info(f"Initial state: armed={initial_state.is_armed}, mode={initial_state.fly_mode}")
        
        # 模拟状态变化（通常这些会从实际的MavLink消息中解析）
        from pymavlink.dialects.v20 import common as mavlink2
        
        # 创建模拟心跳消息
        heartbeat_msg = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_QUADROTOR,
            autopilot=mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink2.MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=(3 << (8 * 3)),  # 定点模式
            system_status=mavlink2.MAV_STATE_ACTIVE,
            mavlink_version=2
        )
        
        # 解析消息（模拟接收到数据）
        airplane.parse_state_from_mavlink(heartbeat_msg)
        
        # 检查状态更新
        updated_state = airplane.get_state()
        logger.info(f"Updated state: armed={updated_state.is_armed}, mode={updated_state.fly_mode}")
        
        # 测试GPS位置模拟
        gps_msg = mavlink2.MAVLink_global_position_int_message(
            time_boot_ms=12345,
            lat=int(39.9042 * 1e7),  # 北京纬度
            lon=int(116.4074 * 1e7),  # 北京经度
            alt=int(50 * 1e3),  # 高度50米
            relative_alt=int(50 * 1e3),
            vx=0,
            vy=0,
            vz=0,
            hdg=0
        )
        
        airplane.parse_state_from_mavlink(gps_msg)
        
        # 检查GPS位置
        gps_pos = airplane.get_gps_pos()
        if gps_pos:
            logger.info(f"GPS position: lat={gps_pos['lat']:.6f}, lon={gps_pos['lon']:.6f}, alt={gps_pos['alt']:.2f}")
        
        # 测试缓存的数据包
        cached_heartbeat = airplane.get_cached_packet(mavlink2.MAVLINK_MSG_ID_HEARTBEAT)
        if cached_heartbeat:
            logger.info(f"Cached heartbeat timestamp: {cached_heartbeat.timestamp}")
            
        cached_gps = airplane.get_cached_packet(mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        if cached_gps:
            logger.info(f"Cached GPS timestamp: {cached_gps.timestamp}")
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        test_state_changes()
        logger.info("State management test completed")
    except Exception as e:
        logger.error(f"Error in state management test: {e}")
    finally:
        manager.stop()
        loop.close()

def test_multiple_airplanes():
    """测试多无人机管理"""
    logger.info("=== 测试多无人机管理 ===")
    
    manager = create_manager()
    manager.init()
    
    def test_multiple():
        # 创建多个无人机
        airplanes = []
        for i in range(1, 6):  # 创建5个无人机，ID为1-5
            airplane = manager.get_airplane(i)
            airplanes.append(airplane)
            
        logger.info(f"Created {len(airplanes)} airplanes")
        
        # 发送控制命令给不同的无人机
        airplanes[0].arm()  # 无人机1解锁
        airplanes[1].takeoff(15.0)  # 无人机2起飞到15米
        airplanes[2].return_to_launch()  # 无人机3返航
        airplanes[3].land()  # 无人机4降落
        airplanes[4].disarm()  # 无人机5锁定
        
        # 获取所有无人机列表
        airplane_list = manager.get_airplane_list()
        logger.info(f"Total airplanes in manager: {len(airplane_list)}")
        
        # 测试移除无人机
        removed = manager.remove_airplane(3)
        logger.info(f"Removed airplane 3: {removed}")
        
        updated_list = manager.get_airplane_list()
        logger.info(f"Airplanes after removal: {len(updated_list)}")
        
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        test_multiple()
        logger.info("Multiple airplanes test completed")
    except Exception as e:
        logger.error(f"Error in multiple airplanes test: {e}")
    finally:
        manager.stop()
        loop.close()

def main():
    """主测试函数"""
    logger.info("开始测试无人机管理系统")
    
    try:
        # 运行各项测试
        test_basic_functionality()
        time.sleep(1)
        
        test_airplane_state_management()
        time.sleep(1)
        
        test_multiple_airplanes()
        time.sleep(1)
        
        test_with_serial_simulation()
        
        logger.info("所有测试完成")
        
    except Exception as e:
        logger.error(f"测试过程中出现错误: {e}")

if __name__ == '__main__':
    main()
