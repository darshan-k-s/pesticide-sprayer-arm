from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_automation'

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
    description='Automated task management for leaf detection and robot arm movement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automation_orchestrator = task_automation.automation_orchestrator:main',
        ],
    },
)

