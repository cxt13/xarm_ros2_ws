from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'foundationpose_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Make sure to include your launch file
        ('share/' + package_name + '/launch', ['launch/foundationpose_grasp.launch.py']),

        # --- ADD THESE TWO LINES ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=['setuptools','geometry_msgs', 'tf2_geometry_msgs', 'rclpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Bridge between FoundationPose and xArm for grasping.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line makes 'ros2 run foundationpose_bridge foundationpose_grasp_bridge' work
            # NEW line
            'foundationpose_grasp_bridge = foundationpose_bridge.foundationpose_grasp_bridge:main',
        ],
    },
)