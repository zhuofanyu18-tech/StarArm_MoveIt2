from setuptools import find_packages, setup

package_name = 'realsense_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            ['config/best.onnx', 'config/d435i_color.yaml'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/pepper_rou_detector.launch.py'],
        ),
        ('share/' + package_name, ['requirements-perception.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu',
    maintainer_email='zhuofanyu18@gmail.com',
    description='Pepper rou detection using YOLO ONNX (GPU/CPU)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pepper_detector = realsense_bringup.pepper_detector:main',
            'pepper_detector_gpu = realsense_bringup.pepper_detector_gpu:main',
        ],
    },
)
