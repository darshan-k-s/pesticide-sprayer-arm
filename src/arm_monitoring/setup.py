from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hao',
    maintainer_email='hao@todo.todo',
    description='UR5e Robot Arm Monitoring Tool - Real-time viewing of joint states and end effector position',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_position_viewer = arm_monitoring.arm_position_viewer:main',
        ],
    },
)


