# handeye_calibration.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ===============================================
    # === VERIFY AND EDIT THESE PARAMETERS ==========
    # ===============================================
    
    # --- Robot Parameters ---
    # IP address of your xArm
    robot_ip = '192.168.1.223'
    # The TF frame for the robot's base
    robot_base_frame = "link_base" 
    # The TF frame for the robot's end-effector (where the camera is mounted)
    robot_effector_frame = "link_tcp" 

    # --- Camera Parameters ---
    # The launch file to start your specific camera
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py' # CHANGE THIS to your camera's launch file
        ])
    )
    # The TF frame of the camera's optical center
    tracking_base_frame = "camera_color_optical_frame"

    # --- Marker Parameters ---
    # The TF frame of the detected marker published by the aruco_node
    tracking_marker_frame = "aruco_single_marker" 
    
    # ===============================================

    # 1. Launch xArm MoveIt
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('xarm_moveit_config'), 'launch'),
            '/xarm5_moveit_realmove.launch.py' # Change xarm5 to your robot DOF
        ]),
        launch_arguments={'robot_ip': robot_ip}.items()
    )

    # 2. Launch Camera Driver
    # (Defined in the parameters section above)

    # 3. Launch ArUco Marker Tracker
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[{
            'camera_frame': tracking_base_frame,
            'marker_size': 0.05, # IMPORTANT: Measure your marker and set size in meters
        }],
        remappings=[
            # Remap the topics to match your camera's output
            ('/camera_info', '/color/camera_info'),
            ('/image', '/color/image_raw'),
        ]
    )

    # 4. Launch the easy_handeye2 calibration node
    easy_handeye_node = Node(
        package="easy_handeye2",
        executable="handeye_calibration",
        namespace="/easy_handeye", # Use a namespace to keep things organized
        parameters=[{
            "robot_base_frame": robot_base_frame,
            "robot_effector_frame": robot_effector_frame,
            "tracking_base_frame": tracking_base_frame,
            "tracking_marker_frame": tracking_marker_frame,
            "calibration_type": "eye_in_hand",
        }],
        output="screen"
    )
    
    # 5. Launch the RViz GUI for easy_handeye2
    easy_handeye_rviz_node = Node(
        package="easy_handeye2",
        executable="handeye_rviz_marker",
        namespace="/easy_handeye",
    )

    return LaunchDescription([
        xarm_moveit_launch,
        camera_launch_file,
        aruco_node,
        easy_handeye_node,
        easy_handeye_rviz_node
    ])