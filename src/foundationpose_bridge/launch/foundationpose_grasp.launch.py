import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # This node publishes the static transform for your EYE-IN-HAND setup.
    # It connects the robot's end-effector to the camera.
    # The values are taken directly from your hand_eye_calibration.yaml file.
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        
        # Arguments are: x y z qx qy qz qw parent_frame child_frame
        arguments=[
            '0.06978', '0.03581', '0.01679',      # Translation (x, y, z)
            '-0.00205', '0.00854', '-0.71811', '0.69586', # Rotation (qx, qy, qz, qw)
            'link_eef',                           # Parent Frame: The robot's end-effector
            'camera_color_optical_frame'          # Child Frame: The camera
        ]
    )

    # --- Robot-Specific Parameters ---
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.223',
        description='IP address of the xArm controller'
    )
    robot_ip = LaunchConfiguration('robot_ip')

    # --- NEW: Define the path to your calibration file ---
    calibration_file_path = os.path.join(
        get_package_share_directory('foundationpose_bridge'),
        'config',
        'hand_eye_calibration.yaml'
    )



    # --- Launch the xArm Hardware Driver and API ---
    xarm_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_api'), 'launch', 'xarm5_driver.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': 'rich',
            'dof': '5',
            'hw_ns': 'xarm',
            'add_gripper': 'true',
        }.items()
    )

    # --- Launch Your Bridge Node (with a delay) ---
    delayed_bridge_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='foundationpose_bridge',
                executable='foundationpose_grasp_bridge',
                name='foundationpose_grasp_bridge',
                output='screen',
                parameters=[{
                    'foundationpose_topics': ['/Current_OBJ_position_1', '/Current_OBJ_position_2', '/Current_OBJ_position_3'],
                    'n_samples': 20,
                    'robot_base_frame': 'link_base',
                    'robot_ip': robot_ip,
                    # --- NEW: Add the calibration file path to the parameters ---
                    'calibration_file_path': calibration_file_path
                }]
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        xarm_api_launch,
        static_tf_publisher,
        delayed_bridge_node
    ])