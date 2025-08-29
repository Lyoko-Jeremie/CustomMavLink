#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup script for CustomMavLink - 无人机管理系统
"""

from setuptools import setup, find_packages
import os

# 读取README文件作为长描述
def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return ""

# 读取requirements.txt文件
def read_requirements():
    req_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    requirements = []
    if os.path.exists(req_path):
        with open(req_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    requirements.append(line)
    return requirements

# 获取版本号
def get_version():
    version_file = os.path.join(os.path.dirname(__file__), 'version.py')
    if os.path.exists(version_file):
        version_vars = {}
        with open(version_file, 'r', encoding='utf-8') as f:
            exec(f.read(), version_vars)
        return version_vars['__version__']
    return '1.0.0'

setup(
    name='custom-mavlink',
    version=get_version(),
    author='Lyoko-Jeremie',
    author_email='',  # 请填写您的邮箱
    description='基于Python的无人机管理系统，提供MavLink协议通信功能',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/Lyoko-Jeremie/CustomMavLink',
    
    # 包配置
    packages=find_packages(exclude=['tests*', '__pycache__*']),
    py_modules=[
        'airplane_owl02',
        'airplane_manager_owl02', 
        'main',
        'owl02'
    ],
    
    # 依赖
    install_requires=read_requirements(),
    
    # Python版本要求
    python_requires='>=3.7',
    
    # 分类信息
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
        'Operating System :: OS Independent',
    ],
    
    # 关键词
    keywords='mavlink drone uav aircraft management serial communication',
    
    # 项目URLs
    project_urls={
        'Bug Reports': 'https://github.com/Lyoko-Jeremie/CustomMavLink/issues',
        'Source': 'https://github.com/Lyoko-Jeremie/CustomMavLink',
    },
    
    # 入口点 - 如果需要命令行工具
    entry_points={
        'console_scripts': [
            'custom-mavlink=airplane_control_example:main',
        ],
    },
    
    # 包含的数据文件
    include_package_data=True,
    package_data={
        '': ['*.md', '*.txt', '*.rst'],
    },
    
    # 额外文件
    data_files=[
        ('', ['README.md', 'requirements.txt']),
    ],
    
    # 开发依赖
    extras_require={
        'dev': [
            'pytest>=6.0',
            'pytest-asyncio',
            'black',
            'flake8',
            'mypy',
        ],
        'test': [
            'pytest>=6.0',
            'pytest-asyncio',
        ],
    },
    
    # zip_safe
    zip_safe=False,
)
