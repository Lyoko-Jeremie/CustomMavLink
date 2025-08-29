#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速构建脚本 - build.py
使用方法: python build.py
"""

import os
import sys
import shutil
import subprocess

def run_command(cmd, description):
    """运行命令并显示结果"""
    print(f"\n🔄 {description}...")
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        print(f"✅ {description} 成功")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} 失败:")
        print(f"错误: {e}")
        if e.stdout:
            print(f"输出: {e.stdout}")
        if e.stderr:
            print(f"错误输出: {e.stderr}")
        return False

def clean_build():
    """清理构建文件"""
    print("🧹 清理构建文件...")
    dirs_to_clean = ['build', 'dist', '*.egg-info']
    for pattern in dirs_to_clean:
        for item in os.listdir('.'):
            if item.startswith('custom_mavlink') and item.endswith('.egg-info'):
                if os.path.isdir(item):
                    shutil.rmtree(item)
                    print(f"删除目录: {item}")
    
    for dirname in ['build', 'dist']:
        if os.path.exists(dirname):
            shutil.rmtree(dirname)
            print(f"删除目录: {dirname}")

def check_dependencies():
    """检查依赖"""
    print("🔍 检查依赖...")
    try:
        import setuptools, wheel
        print("✅ setuptools 和 wheel 已安装")
        return True
    except ImportError:
        print("❌ 缺少依赖，正在安装...")
        return run_command("pip install setuptools wheel", "安装依赖")

def build_package():
    """构建包"""
    if not check_dependencies():
        return False
    
    clean_build()
    
    # 构建包
    if not run_command("python setup.py sdist bdist_wheel", "构建包"):
        return False
    
    # 显示结果
    if os.path.exists('dist'):
        print("\n🎉 构建成功！生成的文件:")
        for file in os.listdir('dist'):
            file_path = os.path.join('dist', file)
            size = os.path.getsize(file_path)
            print(f"  📦 {file} ({size:,} bytes)")
        
        print("\n📋 后续操作:")
        print("1. 安装本地包:")
        wheel_files = [f for f in os.listdir('dist') if f.endswith('.whl')]
        if wheel_files:
            print(f"   pip install dist/{wheel_files[0]}")
        
        print("\n2. 检查包:")
        print("   pip install twine")
        print("   twine check dist/*")
        
        print("\n3. 上传到PyPI:")
        print("   twine upload dist/*")
        
        return True
    else:
        print("❌ 构建失败，未找到 dist 目录")
        return False

if __name__ == "__main__":
    print("=" * 50)
    print("🚀 CustomMavLink 包构建脚本")
    print("=" * 50)
    
    if build_package():
        print("\n✅ 构建完成！")
        sys.exit(0)
    else:
        print("\n❌ 构建失败！")
        sys.exit(1)
