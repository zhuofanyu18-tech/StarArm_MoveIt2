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
            ['launch/pepper_detector.launch.py', 'launch/pepper_detector_gpu.launch.py'],
        ),
        ('share/' + package_name + '/rviz', ['rviz/pepper_detector.rviz']),
        ('share/' + package_name, ['requirements-perception.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yu',
    maintainer_email='zhuofanyu18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle_tracker = realsense_bringup.circle_tracker:main',
            'pepper_detector = realsense_bringup.pepper_detector:main',
            'pepper_point_transformer = realsense_bringup.pepper_point_transformer:main',
            'pepper_grasp_selector = realsense_bringup.pepper_grasp_selector:main',
            'pepper_bing_confirmer = realsense_bringup.pepper_bing_confirmer:main',
            'pepper_confirmed_transformer = realsense_bringup.pepper_confirmed_transformer:main',
            'pepper_detector_gpu = realsense_bringup.pepper_detector_gpu:main',
        ],
    },
)
