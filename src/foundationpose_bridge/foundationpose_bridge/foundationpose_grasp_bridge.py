#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import tf2_ros
import time
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from xarm_msgs.srv import MoveCartesian, SetInt16, SetInt16ById
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.action import ExecuteTrajectory
from xarm_msgs.msg import RobotMsg
from scipy.spatial.transform import Rotation as R
from threading import Lock

# Constants for robot positions (in meters and radians)
SAFE_TRAVEL_POSE = [0.1835, 0.002, 0.444, math.radians(180.0), 0.0, 0.0]
#INSPECTION_POSE = [0.008, 0.0, 0.331, math.pi, -math.radians(30.9), 0.0]
INSPECTION_POSE = [0.2472, -0.0177, 0.2648, math.radians(180.0),math.radians(1.6),0.0]
PARALLEL_POSE = [0.472, 0.0, 0.360, math.radians(180.0),0.0,0.0]
DROP_BOX_PRE_POSE = [0.0174, -0.2312, 0.426, math.pi, -math.radians(1.0), -math.radians(85.7)]
DROP_BOX_POSE = [0.0174, -0.2312, 0.208, math.pi, -math.radians(1.0), -math.radians(85.7)]

# Gripper positions IN METERS (standard xArm gripper range is ~0 to 0.085m)
GRIPPER_OPEN_METERS = 0.085
GRIPPER_CLOSE_METERS = 0.046 # Default, will be overridden by object config

# Object-specific grasp configurations
OBJ_CONFIGS = {
    "DEFAULT": {
        "x_offset": 0.0, 
        "y_offset": 0.0, 
        "z_pre_grasp": 0.1, 
        "z_grasp": 0.02, 
        "gripper_width": GRIPPER_CLOSE_METERS # Use the meters constant
    }
}


class FoundationPoseGraspBridge(Node):
    """
    A ROS2 node to bridge FoundationPose detections to an xArm robot for grasping.
    """
    def __init__(self):
        super().__init__('foundationpose_grasp_bridge')
        self.get_logger().info("🚀 Initializing FoundationPose Grasp Bridge...")

        # Declare and get parameters
        self.declare_parameter('foundationpose_topics', ['/Current_OBJ_position_1', '/Current_OBJ_position_2', '/Current_OBJ_position_3', '/Current_OBJ_position_4'])
        self.declare_parameter('n_samples', 15)
        self.declare_parameter('robot_base_frame', 'link_base')
        self.foundationpose_topics = self.get_parameter('foundationpose_topics').get_parameter_value().string_array_value
        self.n_samples = self.get_parameter('n_samples').get_parameter_value().integer_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State management
        self.object_samples = {}
        self.robot_is_moving = True
        self.state_lock = Lock()
        self.target_grasp_pose = None

        # --- Robot Control Clients ---
        self.get_logger().info("Connecting to xArm services and actions...")
        self.motion_enable_client = self.create_client(SetInt16ById, "/xarm/motion_enable") 
        self.set_mode_client = self.create_client(SetInt16, "/xarm/set_mode")
        self.set_state_client = self.create_client(SetInt16, "/xarm/set_state")
        self.move_line_client = self.create_client(MoveCartesian, "/xarm/set_position")
        self.gripper_action_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action')

        # Wait for essential communication servers
        self.wait_for_servers()

        # --- Subscriptions ---
        self.create_subscription(RobotMsg, '/xarm/robot_states', self.robot_state_callback, 10)
        for i, topic in enumerate(self.foundationpose_topics):
            object_id = i + 1
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, obj_id=object_id: self.pose_callback(msg, obj_id), 10
            )
            self.get_logger().info(f"Subscribed to '{topic}' for Object ID {object_id}")

        self.get_logger().info("Starting robot setup and main control loop...")
        self.robot_setup_and_main_loop()

    def wait_for_servers(self):
        self.get_logger().info("Waiting for essential communication servers...")
        
        # Wait for ACTION server
        self.get_logger().info("Waiting for Gripper Action Server...")
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Gripper action server not available. Shutting down.")
            raise SystemExit("Required ROS action server not found.")
        self.get_logger().info("Action Server 'Gripper Control' is available. ✅")

        # Wait for SERVICE servers
        services = {
            "/xarm/motion_enable": self.motion_enable_client,
            "/xarm/set_mode": self.set_mode_client,
            "/xarm/set_state": self.set_state_client,
            "/xarm/set_position": self.move_line_client,
        }
        for name, client in services.items():
            self.get_logger().info(f"Waiting for Service '{name}'...")
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service '{name}' not available. Shutting down.")
                raise SystemExit("Required ROS service not found.")
            self.get_logger().info(f"Service '{name}' is available. ✅")

    def robot_state_callback(self, msg):
        with self.state_lock:
            self.robot_is_moving = (msg.state != 4)

    def pose_callback(self, msg, object_id):
        with self.state_lock:
            if self.robot_is_moving:
                return

        if object_id not in self.object_samples:
            self.object_samples[object_id] = []
        self.object_samples[object_id].append(msg)

        if len(self.object_samples[object_id]) >= self.n_samples:
            self.get_logger().info(f"Collected {self.n_samples} samples for Object ID {object_id}. Processing...")
            self.process_and_transform_pose(object_id)
            self.object_samples[object_id] = []

    def process_and_transform_pose(self, object_id):
        avg_pose = self.average_poses(self.object_samples[object_id])
        if not avg_pose: return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame, avg_pose.header.frame_id, rclpy.time.Time()
            )
            pose_in_base_frame = do_transform_pose(avg_pose.pose, transform)

            if self.is_graspable(pose_in_base_frame):
                self.get_logger().info(f"✅ Object ID {object_id} is graspable. Setting as target.")
                base_frame_pose_stamped = PoseStamped()
                base_frame_pose_stamped.header.frame_id = self.robot_base_frame
                base_frame_pose_stamped.pose = pose_in_base_frame
                self.target_grasp_pose = base_frame_pose_stamped
            else:
                self.get_logger().warning(f"❌ Object ID {object_id} is NOT graspable (bad roll angle).")
        except TransformException as ex:
            self.get_logger().error(f"Could not transform pose: {ex}")

    def average_poses(self, poses):
        if not poses: return None
        avg_pose = PoseStamped()
        avg_pose.header = poses[0].header
        avg_pose.pose.position.x = sum(p.pose.position.x for p in poses) / len(poses)
        avg_pose.pose.position.y = sum(p.pose.position.y for p in poses) / len(poses)
        avg_pose.pose.position.z = sum(p.pose.position.z for p in poses) / len(poses)
        quats = [[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in poses]
        avg_quat = R.from_quat(quats).mean().as_quat()
        avg_pose.pose.orientation.x, avg_pose.pose.orientation.y, avg_pose.pose.orientation.z, avg_pose.pose.orientation.w = avg_quat
        return avg_pose

    def is_graspable(self, pose_msg):
        q = pose_msg.orientation
        roll, _, _ = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        return abs(abs(roll) - 180) < 45

    def call_service(self, client, request, service_name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            self.get_logger().error(f"Service call to {service_name} timed out.")
            return None
        if hasattr(future.result(), 'ret') and future.result().ret != 0:
            self.get_logger().error(f"Service {service_name} failed: {future.result().message}")
            return None
        self.get_logger().info(f"Service '{service_name}' call successful.")
        return future.result()
    
    def move_robot_line(self, pose):
        req = MoveCartesian.Request()
        req.pose = pose
        req.speed = 80.0  # Speed in mm/s
        req.acc = 3000.0   # Acceleration in mm/s^2
        return self.call_service(self.move_line_client, req, 'set_position')

    # CORRECTED and UNIFIED move_gripper function
    def move_gripper(self, width_meters):
        """Sends a goal to the gripper action server with a target width in meters."""
        self.get_logger().info(f"Sending gripper goal: open to {width_meters * 1000:.1f} mm...")
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = width_meters
        goal_msg.command.max_effort = 100.0  # Optional: set a max effort

        self.gripper_action_client.send_goal_async(goal_msg)
        # For simplicity, we don't wait for the result here, but you could add that.
        self.get_logger().info("Gripper goal sent.")
    
    def set_robot_ready(self):
        """Enable motion, set mode to 0, and state to 0."""
        self.call_service(self.motion_enable_client, SetInt16ById.Request(id=8, data=1), 'motion_enable')
        self.call_service(self.set_mode_client, SetInt16.Request(data=0), 'set_mode')
        self.call_service(self.set_state_client, SetInt16.Request(data=0), 'set_state')

    def execute_grasp_sequence(self):
        with self.state_lock:
            if self.target_grasp_pose is None: return
            self.robot_is_moving = True
            pose_to_grasp = self.target_grasp_pose
            self.target_grasp_pose = None

        self.get_logger().info("=== STARTING GRASP SEQUENCE ===")
        config = OBJ_CONFIGS.get("DEFAULT")
        p = pose_to_grasp.pose.position
        q = pose_to_grasp.pose.orientation
        roll, pitch, yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        
        pre_grasp_pose = [p.x + config['x_offset'], p.y + config['y_offset'], p.z + config['z_pre_grasp'], roll, pitch, yaw]
        grasp_pose = [p.x + config['x_offset'], p.y + config['y_offset'], p.z + config['z_grasp'], roll, pitch, yaw]
        
        #self.move_robot_line(PARALLEL_POSE)
        if not self.move_robot_line(PARALLEL_POSE):
            self.get_logger().error("Failed to move to PARALLEL_POSE. Aborting grasp sequence.")
            # You might want to reset robot_is_moving to false here
            with self.state_lock:
                self.robot_is_moving = False
            return # This exits the function

        #self.move_robot_line(pre_grasp_pose)
        if not self.move_robot_line(pre_grasp_pose):
            self.get_logger().error("Failed to move to pre_grasp_pose. Aborting grasp sequence.")
            with self.state_lock:
                self.robot_is_moving = False
            return

        #self.move_robot_line(grasp_pose)
        if not self.move_robot_line(grasp_pose):
            self.get_logger().error("Failed to move to grasp_pose. Aborting grasp sequence.")
            with self.state_lock:
                self.robot_is_moving = False
            return

        self.move_gripper(config['gripper_width']) # Use width in meters
        time.sleep(1.5)
        self.move_robot_line(pre_grasp_pose)
        self.move_robot_line(PARALLEL_POSE)
        
        self.move_robot_line(DROP_BOX_PRE_POSE)
        self.move_robot_line(DROP_BOX_POSE)
        self.move_gripper(GRIPPER_OPEN_METERS) # Use width in meters
        time.sleep(1.5)

        self.move_robot_line(DROP_BOX_PRE_POSE)
        self.move_robot_line(INSPECTION_POSE)

        self.get_logger().info("✅ Grasp sequence complete.")
        with self.state_lock:
            self.robot_is_moving = False

    def robot_setup_and_main_loop(self):
        self.get_logger().info("Setting up robot...")
        self.set_robot_ready()
        self.move_gripper(GRIPPER_OPEN_METERS) # Use width in meters

        # --- NEW TWO-STEP MOVE ---
        # 1. First, move to the safe intermediate pose
        self.get_logger().info("Moving to safe travel pose...")
        if not self.move_robot_line(SAFE_TRAVEL_POSE):
            self.get_logger().error("Failed to move to SAFE_TRAVEL_POSE. Aborting setup.")
            return # Stop if it fails

        # 2. Now, move from the safe pose to your final inspection pose
        self.get_logger().info("Moving to final inspection pose...")
        if not self.move_robot_line(INSPECTION_POSE):
            self.get_logger().error("Failed to move to INSPECTION_POSE. Aborting setup.")
            return # Stop if it fails

        
        with self.state_lock:
            self.robot_is_moving = False
        self.get_logger().info("Robot is ready. Listening for objects...")

        self.create_timer(1.0, self.execute_grasp_sequence)

def main(args=None):
    rclpy.init(args=args)
    node = FoundationPoseGraspBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()