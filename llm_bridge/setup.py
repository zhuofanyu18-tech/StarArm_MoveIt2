import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'llm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu',
    maintainer_email='zhuofanyu18@gmail.com',
    description='ROS2 bridge node to llama.cpp server for LLM chat',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'llm_bridge_node = llm_bridge.llm_bridge_node:main',
        ],
    },
)
