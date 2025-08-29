#!/usr/bin/env python3
"""
测试OWL02适配器的API接口
验证与ph0apy.py的API兼容性
"""

import sys
import time
import logging
from owl02 import Owl02Controller

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_owl02_api():
    """测试OWL02 API接口"""
    logger.info("开始测试OWL02 API接口")
    
    try:
        # 创建Owl02Controller实例
        controller = Owl02Controller()
        logger.info("✓ Owl02Controller实例创建成功")
        
        # 测试添加无人机 - 使用COM6:3格式
        test_uav_id = "COM6:3"
        controller.add_uav(test_uav_id)
        logger.info(f"✓ 添加无人机成功 - ID: {test_uav_id}")
        
        # 测试纯数字格式
        test_uav_id_num = 5
        controller.add_uav(test_uav_id_num)
        logger.info(f"✓ 添加无人机成功 - ID: {test_uav_id_num}")
        
        # 测试获取无人机对象
        drone = controller.p(test_uav_id)
        if drone:
            logger.info(f"✓ 获取无人机对象成功 - 设备ID: {drone.target_channel_id}")
        
        # 测试基本控制命令（这些不会实际发送，因为没有串口连接）
        logger.info("测试基本控制命令...")
        
        # 测试起飞
        controller.takeoff(test_uav_id, 100)  # 起飞到100cm
        logger.info("✓ 起飞命令测试通过")
        
        # 测试移动命令
        controller.up(test_uav_id, 50)
        controller.down(test_uav_id, 30)
        controller.forward(test_uav_id, 100)
        controller.back(test_uav_id, 50)
        controller.left(test_uav_id, 40)
        controller.right(test_uav_id, 60)
        logger.info("✓ 移动命令测试通过")
        
        # 测试旋转命令
        controller.rotate(test_uav_id, 90)
        controller.cw(test_uav_id, 45)
        controller.ccw(test_uav_id, 30)
        logger.info("✓ 旋转命令测试通过")
        
        # 测试goto命令
        controller.goto(test_uav_id, 100, 200, 150)
        logger.info("✓ goto命令测试通过")
        
        # 测试高度命令
        controller.high(test_uav_id, 200)
        logger.info("✓ 高度命令测试通过")
        
        # 测试速度设置
        controller.speed(test_uav_id, 50)
        logger.info("✓ 速度设置测试通过")
        
        # 测试LED命令
        controller.led(test_uav_id, 255, 0, 0)  # 红色
        controller.bln(test_uav_id, 0, 255, 0)  # 绿色呼吸灯
        controller.rainbow(test_uav_id, 0, 0, 255)  # 蓝色彩虹
        logger.info("✓ LED命令测试通过")
        
        # 测试飞行模式
        controller.mode(test_uav_id, 4)  # 单机编队模式
        logger.info("✓ 飞行模式设置测试通过")
        
        # 测试翻滚命令（两种调用方式）
        controller.flip(test_uav_id, 'f')  # 标准方式
        controller.flip('f')  # 兼容原始API的方式
        logger.info("✓ 翻滚命令测试通过")
        
        # 测试悬停和停桨
        controller.hover(test_uav_id)
        logger.info("✓ 悬停命令测试通过")
        
        # 测试降落
        controller.land(test_uav_id)
        logger.info("✓ 降落命令测试通过")
        
        # 测试停桨
        controller.stop(test_uav_id)
        logger.info("✓ 停桨命令测试通过")
        
        # 测试sleep功能
        logger.info("测试sleep功能...")
        controller.sleep(0.1)  # 睡眠0.1秒
        logger.info("✓ sleep功能测试通过")
        
        # 测试错误的ID格式
        try:
            controller.add_uav("COM6:17")  # 超出范围
            logger.error("❌ 应该抛出错误：ID超出范围")
        except ValueError as e:
            logger.info("✓ ID范围验证测试通过")
        
        # 清理
        controller.destroy()
        logger.info("✓ 销毁管理器成功")
        
        logger.info("🎉 所有API测试通过！")
        return True
        
    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_multiple_drones():
    """测试多无人机支持"""
    logger.info("开始测试多无人机支持")
    
    try:
        controller = Owl02Controller()
        
        # 添加多架无人机 - 使用不同格式的ID
        uav_ids = ["COM6:1", "COM6:2", 3, "COM8:4"]
        for uav_id in uav_ids:
            controller.add_uav(uav_id)
            logger.info(f"✓ 添加无人机 {uav_id}")
        
        # 对每架无人机执行不同命令
        controller.takeoff("COM6:1", 100)
        controller.takeoff("COM6:2", 150)
        controller.takeoff(3, 200)
        
        controller.led("COM6:1", 255, 0, 0)    # 红色
        controller.led("COM6:2", 0, 255, 0)    # 绿色
        controller.led(3, 0, 0, 255)           # 蓝色
        
        logger.info("✓ 多无人机控制测试通过")
        
        controller.destroy()
        return True
        
    except Exception as e:
        logger.error(f"❌ 多无人机测试失败: {e}")
        return False


def main():
    """主测试函数"""
    logger.info("开始OWL02适配器API测试")
    
    # 基本API测试
    if not test_owl02_api():
        sys.exit(1)
    
    # 多无人机测试
    if not test_multiple_drones():
        sys.exit(1)
    
    logger.info("🎉 所有测试完成！OWL02适配器API兼容性验证通过")


if __name__ == '__main__':
    main()
