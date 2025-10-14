"""
无人机飞行测试脚本 - 简化版
测试 AirplaneManagerOwl02 的基本操作：初始化、起飞、飞行、降落
"""
import time
import logging
from airplane_manager_owl02 import create_manager

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    logger.info("=" * 60)
    logger.info("无人机简易飞行测试")
    logger.info("=" * 60)

    # 创建管理器
    logger.info("\n步骤 1: 创建管理器")
    manager = create_manager()
    manager.init()
    logger.info("✓ 管理器创建成功")

    # 获取无人机对象
    logger.info("\n步骤 2: 初始化无人机 (ID=1)")
    drone = manager.get_airplane(1)
    logger.info("✓ 无人机初始化成功")
    time.sleep(1)

    try:
        # 解锁
        logger.info("\n解锁无人机")
        drone.arm()
        time.sleep(2)

        # 起飞
        logger.info("\n起飞到 1.5 米")
        drone.takeoff(150)
        time.sleep(5)

        # 前进
        logger.info("\n前进")
        drone.forward(100)
        time.sleep(3)

        # 后退
        logger.info("\n后退")
        drone.back(100)
        time.sleep(3)

        # goto
        logger.info("\ngoto")
        drone.goto(100, 100, 150)
        time.sleep(5)

        # 降落
        logger.info("\n降落")
        drone.land()
        time.sleep(8)

        # 上锁
        logger.info("\n上锁无人机")
        drone.disarm()
        time.sleep(2)

        logger.info("\n" + "=" * 60)
        logger.info("✓ 测试完成！")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"\n✗ 测试失败: {e}")

    finally:
        # 清理资源
        manager.stop()
        logger.info("✓ 管理器已停止")
