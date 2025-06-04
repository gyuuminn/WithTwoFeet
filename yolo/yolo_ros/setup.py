from setuptools import find_packages, setup

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='two',
    maintainer_email='two@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo1 = yolo_ros.yolo1:main',
            'mode = yolo_ros.mode_mux:main',
            'joy_twist = yolo_ros.joy_twist:main',
            'joy_twist_teleop = yolo_ros.joy_twist_teleop:main',
            'motor = yolo_ros.motor:main',
            'motor_bridge = yolo_ros.motor_bridge:main',
        ],
    },
)
