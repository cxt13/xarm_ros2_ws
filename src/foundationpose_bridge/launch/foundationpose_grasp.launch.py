import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Static TF: end-effector → camera (from hand_eye_calibration.yaml)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        arguments=[
            '0.06978035792908464', '0.035818571378670123', '0.01679021439600517',
            '-0.002053764557758081', '0.008549615327250404', '-0.7181193669041934', '0.6958643984326385',
            'camera_color_optical_frame', 'link_eef'
        ]
    )

    # 2) Robot IP argument for xArm driver
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.223',
        description='IP address of the xArm controller'
    )
    robot_ip = LaunchConfiguration('robot_ip')

    # 3) Hand–eye calibration YAML path
    calibration_file_path = os.path.join(
        get_package_share_directory('foundationpose_bridge'),
        'config',
        'hand_eye_calibration.yaml'
    )

    # 4) Launch the xArm hardware driver & API
    xarm_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xarm_api'),
                'launch',
                'xarm5_driver.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip':     robot_ip,
            'report_type':  'rich',
            'dof':          '5',
            'hw_ns':        'xarm',
            'add_gripper':  'true',
        }.items()
    )

    # 5) Delay the bridge node until the hardware driver is up
    delayed_bridge_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='foundationpose_bridge',
                executable='foundationpose_grasp_bridge',
                name='foundationpose_grasp_bridge',
                output='screen',
                remappings=[
                    ('/xarm_gripper/gripper_action', '/xarm_driver/gripper_action'),
                ],
                parameters=[{
                    # exactly these three FoundationPose topics:
                    'foundationpose_topics': [
                        '/Current_OBJ_position_1',
                        '/Current_OBJ_position_2',
                        '/Current_OBJ_position_3'
                    ],
                    'n_samples':             20,
                    # must match your robot’s TF frame
                    'robot_base_frame':      'link_base',
                    # pass through the robot_ip arg
                    'robot_ip':              robot_ip,
                    # hand–eye calibration file location
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
