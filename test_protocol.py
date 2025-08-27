#!/usr/bin/env python3
"""
测试自定义包协议的封装和解析功能
"""

import struct
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import wrap_packet, PacketParser, HEADER1, HEADER2, TAIL

def test_packet_wrapping():
    """测试包封装功能"""
    print("=== 测试包封装功能 ===")
    
    # 测试数据
    test_data = b"Hello, MavLink!"
    device_id = 5
    
    # 封装数据包
    packet = wrap_packet(device_id, test_data)
    
    print(f"原始数据: {test_data}")
    print(f"设备ID: {device_id}")
    print(f"封装后的包: {packet.hex()}")
    print(f"包长度: {len(packet)}")
    
    # 手动验证包格式
    expected_format = f"{HEADER1:02X} {HEADER2:02X} {device_id:02X} {len(test_data):02X}"
    actual_format = " ".join(f"{b:02X}" for b in packet[:4])
    print(f"包头格式 (期望): {expected_format}")
    print(f"包头格式 (实际): {actual_format}")
    
    # 验证帧尾
    print(f"帧尾 (期望): {TAIL:02X}")
    print(f"帧尾 (实际): {packet[-1]:02X}")
    
    return packet

def test_packet_parsing():
    """测试包解析功能"""
    print("\n=== 测试包解析功能 ===")
    
    # 创建测试数据包
    test_data = b"Test MavLink Data"
    device_id = 3
    packet = wrap_packet(device_id, test_data)
    
    # 创建解析器
    parser = PacketParser()
    
    # 添加数据并解析
    parser.add_data(packet)
    parsed_packets = parser.parse_packets()
    
    print(f"原始数据: {test_data}")
    print(f"解析到的包数量: {len(parsed_packets)}")
    
    if parsed_packets:
        parsed_packet = parsed_packets[0]
        print(f"解析出的设备ID: {parsed_packet['device_id']}")
        print(f"解析出的数据: {parsed_packet['payload']}")
        print(f"数据是否匹配: {parsed_packet['payload'] == test_data}")

def test_packet_fragmentation():
    """测试包切割和缓存功能"""
    print("\n=== 测试包切割和缓存功能 ===")
    
    # 创建多个测试数据包
    packets_data = [
        (1, b"Packet 1"),
        (2, b"Packet 2 with more data"),
        (3, b"Packet 3")
    ]
    
    # 生成完整的数据流
    data_stream = bytearray()
    for device_id, data in packets_data:
        packet = wrap_packet(device_id, data)
        data_stream.extend(packet)
    
    print(f"总数据流长度: {len(data_stream)} 字节")
    
    # 创建解析器
    parser = PacketParser()
    
    # 模拟分片接收数据
    chunk_size = 5  # 每次只接收5个字节
    received_packets = []
    
    for i in range(0, len(data_stream), chunk_size):
        chunk = data_stream[i:i + chunk_size]
        print(f"接收数据片段 {i//chunk_size + 1}: {chunk.hex()}")
        
        parser.add_data(chunk)
        parsed = parser.parse_packets()
        received_packets.extend(parsed)
        
        if parsed:
            print(f"  -> 解析到 {len(parsed)} 个包")
    
    print(f"\n总共解析到 {len(received_packets)} 个包:")
    for i, packet in enumerate(received_packets):
        print(f"  包 {i+1}: 设备ID={packet['device_id']}, 数据={packet['payload']}")

def test_invalid_data_handling():
    """测试无效数据处理"""
    print("\n=== 测试无效数据处理 ===")
    
    # 创建包含无效数据的数据流
    valid_packet = wrap_packet(5, b"Valid data")
    invalid_data = b"\x12\x34\x56\x78"  # 无效的随机数据
    
    data_stream = invalid_data + valid_packet + b"\xFF\xFE\xFD"  # 前后都有无效数据
    
    print(f"数据流 (包含无效数据): {data_stream.hex()}")
    
    # 解析数据
    parser = PacketParser()
    parser.add_data(data_stream)
    parsed_packets = parser.parse_packets()
    
    print(f"解析到的有效包数量: {len(parsed_packets)}")
    if parsed_packets:
        packet = parsed_packets[0]
        print(f"有效包: 设备ID={packet['device_id']}, 数据={packet['payload']}")

if __name__ == "__main__":
    # 运行所有测试
    test_packet_wrapping()
    test_packet_parsing()
    test_packet_fragmentation()
    test_invalid_data_handling()
    
    print("\n=== 测试完成 ===")
