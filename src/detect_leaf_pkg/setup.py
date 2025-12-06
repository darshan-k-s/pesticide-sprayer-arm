from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detect_leaf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'plantcv', 'opencv-python', 'numpy', 'scipy', 'pyrealsense2', 'PyYAML', 'message_filters'],
    zip_safe=True,
    maintainer='hao',
    maintainer_email='hao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leaf_detection_server = detect_leaf_pkg.leaf_detection_server:main',
            'leaf_detection_client = detect_leaf_pkg.leaf_detection_client:main',
            'leaf_visualization_node = detect_leaf_pkg.leaf_visualization_node:main',
        ],
    },
)
