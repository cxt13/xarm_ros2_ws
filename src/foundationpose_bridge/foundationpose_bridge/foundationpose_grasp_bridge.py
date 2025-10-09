#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
import tf2_ros
import time
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped , PoseArray , Pose
from xarm_msgs.srv import SetInt16, SetInt16ById, MoveCartesian
from rclpy.action import ActionClient 
from rclpy.duration import Duration
from control_msgs.action import GripperCommand
from xarm_msgs.msg import RobotMsg
from scipy.spatial.transform import Rotation as R
from threading import Lock, Thread
from rclpy.executors import MultiThreadedExecutor 

from tf_transformations import quaternion_multiply, quaternion_from_euler

# Constants for robot positions (in meters and radians)
SAFE_TRAVEL_POSE = [0.365, -0.164, 0.175, 3.142, 0.000, 0.677]
INSPECTION_POSE = [ 0.100155762, -0.017557951, 0.319199799, 3.048767003, -0.504771996, 0.016627996 ]
PARALLEL_POSE   = [ 0.298326263, -0.043865643, 0.403236969, -3.121592654, -0.000306009, -0.709524006 ]
DROP_BOX_PRE_POSE = [ 0.002136130, -0.244775543, 0.229525467, 3.121592654, -0.113416993, -1.539234002 ]
DROP_BOX_POSE = [ 0.016955828, -0.346687164, 0.172459763, 3.121592654, -0.099458006, -1.499124992 ]

# Gripper positions IN METERS
GRIPPER_OPEN_METERS = 0.085
GRIPPER_CLOSE_METERS = 0.046


# Object-specific grasp configurations
OBJ_CONFIGS = {
    "DEFAULT": {
        "x_offset": 0.0,
        "y_offset": 0.0,
        "z_pre_grasp": 0.1,
        "z_grasp": 0.02,
        "gripper_width": GRIPPER_CLOSE_METERS
    }
}

class FoundationPoseGraspBridge(Node):
    def __init__(self):
        super().__init__('foundationpose_grasp_bridge')
        self.get_logger().info("🚀 Initializing FoundationPose Grasp Bridge...")


        # Declare and get parameters
        self.declare_parameter('foundationpose_topics', ['/Current_OBJ_position_1', '/Current_OBJ_position_2', '/Current_OBJ_position_3'])
        self.declare_parameter('n_samples', 20)
        self.declare_parameter('robot_base_frame', 'link_base')
        self.foundationpose_topics = self.get_parameter('foundationpose_topics').get_parameter_value().string_array_value
        self.n_samples = self.get_parameter('n_samples').get_parameter_value().integer_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        # TF and state variables setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.object_samples = {}
        self.robot_is_moving = False
        self.state_lock = Lock()
        self.target_grasp_pose = None



        # Create clients and actions
        self.get_logger().info("Connecting to xArm services and actions...")
        self.motion_enable_client = self.create_client(SetInt16ById, "/xarm/motion_enable")
        self.set_mode_client = self.create_client(SetInt16, "/xarm/set_mode")
        self.set_state_client = self.create_client(SetInt16, "/xarm/set_state")
        self.move_line_client = self.create_client(MoveCartesian, "/xarm/set_position")
        self.gripper_action_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action')

        # Create subscriptions
        self.create_subscription(RobotMsg, '/xarm/robot_states', self.robot_state_callback, 10)
        for i, topic in enumerate(self.foundationpose_topics):
            object_id = i + 1
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, obj_id=object_id: self.pose_callback(msg, obj_id), 10
            )
            self.get_logger().info(f"Subscribed to '{topic}' for Object ID {object_id}")

        self.pa_pub = self.create_publisher(PoseArray, '/aggregated_poses', 10)

        # 2) Subscribers for each object pose topic
        self.current_poses = {}  # object_id → latest PoseStamped
        self.subs = []
        for obj_id in [1, 2, 3]:  # update with your real IDs
            topic = f'/Current_OBJ_position_{obj_id}'
            cb = lambda msg, id=obj_id: self.pose_cb(msg, id)
            self.subs.append(self.create_subscription(PoseStamped, topic, cb, 10))

        # 3) Timer to republish the PoseArray at 10 Hz
        self.create_timer(0.1, self.publish_array)

    def _stop_and_capture(self, samples: int = None, settle_secs: float = 0.2, inter_frame_s: float = 0.02):
        if samples is None:
            samples = self.n_samples
        # Best-effort stop; replace with your controller API if different
        try:
            self.controller.stop_motion()
        except Exception:
            try:
                self.controller.set_joint_velocity([0.0] * getattr(self.controller, 'n_joints', 5))
            except Exception:
                self.get_logger().warn("_stop_and_capture: controller stop attempt failed")
        import time
        self.get_logger().info(f"_stop_and_capture: waiting {settle_secs}s for robot to settle")
        time.sleep(settle_secs)

        frames = []
        for _ in range(samples):
            try:
                frames.append(self.detection_buffer.get_latest())
            except Exception:
                frames.append(None)
            time.sleep(inter_frame_s)
        return frames


    def _lookup_latest_transform(self, target_frame: str, source_frame: str, timeout_s: float = 0.5):
        now_zero = Time()  # request latest available transform
        timeout = Duration(seconds=timeout_s)
        if not self.tf_buffer.can_transform(target_frame, source_frame, now_zero, timeout=timeout):
            raise RuntimeError(f"cannot transform {source_frame} -> {target_frame} within {timeout_s}s")
        return self.tf_buffer.lookup_transform(target_frame, source_frame, now_zero, timeout=timeout)



    def pose_cb(self, msg: PoseStamped, object_id: int):
        # store the most recent pose for each object
        self.current_poses[object_id] = msg

    def publish_array(self):
        if not self.current_poses:
            return  # nothing to publish yet

        pa = PoseArray()
        # stamp and frame must match your poses
        first = next(iter(self.current_poses.values()))
        pa.header.frame_id = first.header.frame_id
        pa.header.stamp = first.header.stamp

        # fill the array with the latest poses
        pa.poses = [ps.pose for ps in self.current_poses.values()]
        self.pa_pub.publish(pa)

    def robot_state_callback(self, msg: RobotMsg):
        with self.state_lock:
            if msg.err != 0:
                self.get_logger().warn(f"Robot is in error state! Code: {msg.err}")


    def pose_callback(self, msg, object_id):
        # 0) Ensure the needed transform is available (200 ms timeout)
        # try:
        #     self.tf_buffer.lookup_transform(
        #         self.robot_base_frame,       # target frame
        #         msg.header.frame_id,         # source frame
        #         msg.header.stamp,            # at the message time
        #         timeout=Duration(seconds=0.2)
        #     )
        # except TransformException as e:
        #     self.get_logger().warn(
        #         f"TF not yet available for frame '{msg.header.frame_id}': {e}"
        #     )
        #     return
        try:
            # ask for the latest transform; safe short timeout
            self._lookup_latest_transform(self.robot_base_frame, msg.header.frame_id, timeout_s=0.2)
        except Exception as e:
            self.get_logger().warn(f"TF not yet available for frame '{msg.header.frame_id}': {e}")
            return


        with self.state_lock:
            if self.robot_is_moving or self.target_grasp_pose is not None:
                return

        if object_id not in self.object_samples:
            self.object_samples[object_id] = []
        self.object_samples[object_id].append(msg)

        if len(self.object_samples[object_id]) >= self.n_samples:
            self.get_logger().info(f"Collected {self.n_samples} samples for Object ID {object_id}. Processing...")
            self.process_and_transform_pose(object_id)
            self.object_samples[object_id].clear()


    #def process_and_transform_pose(self, object_id):
        # avg_pose = self.average_poses(self.object_samples[object_id])
        # if not avg_pose:
        #     return

        # try:
        #     now = self.get_clock().now()
        #     transform = self.tf_buffer.lookup_transform(
        #         self.robot_base_frame, avg_pose.header.frame_id, now
        #     )
        #     requested_time = avg_pose.header.stamp  # keep your original time
        #     tol = Duration(seconds=0.1)             # 100 ms tolerance
        #     transform = self.tf_buffer.lookup_transform_full(
        #         self.robot_base_frame,
        #         rclpy.time.Time(),         # target time = latest
        #         avg_pose.header.frame_id,
        #         requested_time,
        #         self.robot_base_frame,     # fixed frame
        #         tol
        #     )


        #     stamped = PoseStamped()
        #     stamped.header = avg_pose.header
        #     stamped.pose = avg_pose.pose

        #     transformed_pose = do_transform_pose(stamped.pose, transform)
        #     pose_in_base_frame_stamped = PoseStamped()
        #     pose_in_base_frame_stamped.header.frame_id = transform.header.frame_id
        #     pose_in_base_frame_stamped.header.stamp = transform.header.stamp
        #     pose_in_base_frame_stamped.pose = transformed_pose

        #     if self.is_graspable(transformed_pose):
        #         self.get_logger().info(f"✅ Object ID {object_id} is graspable.")
        #         base_frame_pose_stamped = PoseStamped()
        #         base_frame_pose_stamped.header.frame_id = self.robot_base_frame
        #         base_frame_pose_stamped.header.stamp = pose_in_base_frame_stamped.header.stamp
        #         base_frame_pose_stamped.pose = transformed_pose

        #         with self.state_lock:
        #             self.target_grasp_pose = base_frame_pose_stamped
        #         return
        #     else:
        #         self.get_logger().warning(
        #             f"❌ Object ID {object_id} is NOT graspable (bad roll angle)."
        #         )

        # except TransformException as ex:
        #     self.get_logger().error(
        #         f"Could not transform pose for object {object_id}: {ex}"
        #     )
    def process_and_transform_pose(self, object_id):
        # 1) Gather averaged pose safely
        avg_pose = self.average_poses(self.object_samples.get(object_id, []))
        if not avg_pose:
            self.get_logger().warning(f"[process] No averaged pose for object {object_id}; skipping")
            return

        # 2) Re-stamp to now to avoid TF buffer extrapolation when robot is moving
        avg_pose.header.stamp = self.get_clock().now().to_msg()

        # 3) Lookup latest transform safely
        try:
            transform = self._lookup_latest_transform(self.robot_base_frame, avg_pose.header.frame_id, timeout_s=0.5)
        except Exception as ex:
            self.get_logger().error(f"Could not get TF for object {object_id}: {ex}")
            return

        # 4) Build PoseStamped and perform the transform using tf2_geometry_msgs
        try:
            # build PoseStamped from avg_pose (avg_pose is already a PoseStamped)
            stamped_in_cam = PoseStamped()
            stamped_in_cam.header = avg_pose.header
            stamped_in_cam.pose = avg_pose.pose

            # transform to base frame (returns PoseStamped)
            transformed_stamped = do_transform_pose(stamped_in_cam, transform)

            # DEBUG: show runtime type
            self.get_logger().info(f"DEBUG: type(transformed_stamped) = {type(transformed_stamped)}")

            # normalize to a geometry_msgs.msg.Pose named `pose`
            if isinstance(transformed_stamped, PoseStamped):
                pose = transformed_stamped.pose
            elif isinstance(transformed_stamped, Pose):
                pose = transformed_stamped
            else:
                raise TypeError(f"Unexpected transform result type: {type(transformed_stamped)}")

            # OPTIONAL: normalize quaternion to avoid downstream IK issues
            qx = float(pose.orientation.x)
            qy = float(pose.orientation.y)
            qz = float(pose.orientation.z)
            qw = float(pose.orientation.w)
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if norm == 0.0:
                raise ValueError("Transformed quaternion has zero norm")
            pose.orientation.x = qx / norm
            pose.orientation.y = qy / norm
            pose.orientation.z = qz / norm
            pose.orientation.w = qw / norm

            # safe logging and checks
            self.get_logger().info(
                f"transformed (base) x={pose.position.x:.3f} "
                f"y={pose.position.y:.3f} z={pose.position.z:.3f}"
            )

            # pass Pose (not PoseStamped) into is_graspable
            if self.is_graspable(pose):
                self.get_logger().info(f"✅ Object ID {object_id} is graspable.")
                base_frame_pose_stamped = PoseStamped()
                base_frame_pose_stamped.header.frame_id = self.robot_base_frame
                base_frame_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                base_frame_pose_stamped.pose = pose

                with self.state_lock:
                    self.target_grasp_pose = base_frame_pose_stamped
                return
            else:
                self.get_logger().warning(f"❌ Object ID {object_id} is NOT graspable (bad roll angle).")

        except Exception as ex:
            self.get_logger().error(f"TF transform failed for object {object_id}: {ex}")
            return



    # def average_poses(self, poses):
    #     if not poses: return None
    #     avg_pose = PoseStamped()
    #     avg_pose.header = poses[0].header
    #     avg_pose.pose.position.x = sum(p.pose.position.x for p in poses) / len(poses)
    #     avg_pose.pose.position.y = sum(p.pose.position.y for p in poses) / len(poses)
    #     avg_pose.pose.position.z = sum(p.pose.position.z for p in poses) / len(poses)
    #     quats = [[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in poses]
    #     avg_quat = R.from_quat(quats).mean().as_quat()
    #     avg_pose.pose.orientation.x, avg_pose.pose.orientation.y, avg_pose.pose.orientation.z, avg_pose.pose.orientation.w = avg_quat
    #     return avg_pose
    def average_poses(self, poses):
        # Return None if no poses
        if not poses:
            return None

        # Ensure we only average valid PoseStamped objects
        valid = [p for p in poses if p is not None]
        if not valid:
            return None

        n = len(valid)

        # Average positions
        avg_pos = np.zeros(3)
        for p in valid:
            avg_pos += np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
        avg_pos /= float(n)

        # Collect quaternions and do a proper quaternion average using Rotation
        quats = np.array([[p.pose.orientation.x,
                           p.pose.orientation.y,
                           p.pose.orientation.z,
                           p.pose.orientation.w] for p in valid])

        # Guard: if any quaternion is zero or NaN, skip averaging
        if not np.isfinite(quats).all() or np.allclose(np.linalg.norm(quats, axis=1), 0.0):
            return None

        # Normalize input quaternions for stability
        quats = quats / np.linalg.norm(quats, axis=1)[:, None]

        # Use Rotation mean to compute average quaternion
        try:
            avg_quat = R.from_quat(quats).mean().as_quat()
        except Exception:
            # Fallback: simple eigenvector-based method (additive then normalize)
            q = quats.sum(axis=0)
            avg_quat = q / np.linalg.norm(q)

        # Build PoseStamped result
        avg_pose = PoseStamped()
        # Use the most recent stamp to reduce TF extrapolation risk
        avg_pose.header = valid[-1].header
        avg_pose.pose.position.x, avg_pose.pose.position.y, avg_pose.pose.position.z = map(float, avg_pos)
        avg_pose.pose.orientation.x = float(avg_quat[0])
        avg_pose.pose.orientation.y = float(avg_quat[1])
        avg_pose.pose.orientation.z = float(avg_quat[2])
        avg_pose.pose.orientation.w = float(avg_quat[3])

        return avg_pose

    def is_graspable(self, pose: 'geometry_msgs.msg.Pose') -> bool:
        q = pose.orientation
        yaw, pitch, roll = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx', degrees=True)
        return abs(pitch) < 60 and abs(roll) < 45


        self.get_logger().info(f"transformed_stamped type: {type(transformed_stamped)}")
        self.get_logger().info(f"transformed_pose type: {type(transformed_pose)}")
        #self.get_logger().info(f"pose in base: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}")
        self.get_logger().info(f"pose in base: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}")





    def call_service(self, client, request, service_name, timeout=20.0):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service '{service_name}' not available. Aborting call.")
            return None

        future = client.call_async(request)
        
        ## --- FIX ---: Wait for the future to complete without spinning.
        # The main executor will handle the completion.
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Service call to '{service_name}' timed out after {timeout}s.")
                return None
            time.sleep(0.1)

        result = future.result()
        if hasattr(result, 'ret') and result.ret != 0:
            self.get_logger().error(f"Service '{service_name}' failed with code {result.ret}: {result.message}")
            return None

        self.get_logger().info(f"Service '{service_name}' call successful.")
        return result

    def move_robot_line(self, pose_in_meters, speed=80.0, acc=500.0):
        pose_in_mm = [p * 1000.0 if i < 3 else p for i, p in enumerate(pose_in_meters)]
        req = MoveCartesian.Request()
        req.pose = pose_in_mm
        req.speed = float(speed)
        req.acc = float(acc)
        return self.call_service(self.move_line_client, req, 'set_position', timeout=30.0)

    def move_gripper(self, width_meters, wait_for_result=True):
        self.get_logger().info(f"Sending gripper goal: set width to {width_meters * 1000:.1f} mm...")
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available.")
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = width_meters
        goal_msg.command.max_effort = 100.0

        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)

        ## --- FIX ---: Wait for the goal acceptance future without spinning.
        while not send_goal_future.done():
            time.sleep(0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        if wait_for_result:
            self.get_logger().info('Gripper goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            ## --- FIX ---: Wait for the result future without spinning.
            while not result_future.done():
                time.sleep(0.1)
            self.get_logger().info('Gripper action finished.')
        else:
            self.get_logger().info('Gripper goal sent.')
        return True

    def execute_grasp_sequence(self):
        with self.state_lock:
            if self.target_grasp_pose is None:
                return
            self.robot_is_moving = True
            pose_to_grasp = self.target_grasp_pose
            self.target_grasp_pose = None

        self.get_logger().info("=== STARTING GRASP SEQUENCE ===")
        config = OBJ_CONFIGS.get("DEFAULT")
        p = pose_to_grasp.pose.position
        q = pose_to_grasp.pose.orientation
        rpy_rad = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx')
        base_orientation = [math.pi, 0.0, rpy_rad[0]]

        pre_grasp_pose = [p.x + config['x_offset'], p.y + config['y_offset'], p.z + config['z_pre_grasp']] + base_orientation
        grasp_pose = [p.x + config['x_offset'], p.y + config['y_offset'], p.z + config['z_grasp']] + base_orientation

        sequence = [
            ("Opening gripper before approach", lambda: self.move_gripper(GRIPPER_OPEN_METERS)),
            ("Moving to pre-grasp pose", lambda: self.move_robot_line(pre_grasp_pose)),
            ("Moving to grasp pose", lambda: self.move_robot_line(grasp_pose, speed=0.1)),
            ("Closing gripper", lambda: self.move_gripper(config['gripper_width'])),
            ("Retreating to pre-grasp pose", lambda: self.move_robot_line(pre_grasp_pose)),
            ("Moving to drop-off pre-pose", lambda: self.move_robot_line(DROP_BOX_PRE_POSE)),
            ("Moving to drop-off pose", lambda: self.move_robot_line(DROP_BOX_POSE)),
            ("Opening gripper to release", lambda: self.move_gripper(GRIPPER_OPEN_METERS)),
            ("Retreating from drop box", lambda: self.move_robot_line(DROP_BOX_PRE_POSE)),
            ("Returning to inspection pose", lambda: self.move_robot_line(INSPECTION_POSE))
        ]

        success = True
        for name, action in sequence:
            self.get_logger().info(f"Executing: {name}...")
            if not action():
                self.get_logger().error(f"Failed step: {name}. Aborting grasp sequence.")
                success = False
                break

        if success:
            self.get_logger().info("✅ Grasp sequence complete.")
        else:
            self.get_logger().error("❌ Grasp sequence failed. Returning to a safe pose.")
            self.move_robot_line(SAFE_TRAVEL_POSE)

        with self.state_lock:
            self.robot_is_moving = False
            self.get_logger().info("Robot is ready. Listening for new objects...")

    def run_setup_and_main_loop(self):
        self.get_logger().info("Waiting for essential services and action servers...")
        servers_to_wait = {
            "Motion Enable Service": self.motion_enable_client,
            "Set Mode Service": self.set_mode_client,
            "Set State Service": self.set_state_client,
            "Move Cartesian Service": self.move_line_client,
        }
        for name, client in servers_to_wait.items():
            if not client.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"'{name}' not available. Shutting down.")
                rclpy.shutdown()
                return

        if not self.gripper_action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Gripper Action Server not available. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info("All services and actions are ready. ✅")

        self.get_logger().info("Setting up robot state...")
        if not self.call_service(self.motion_enable_client, SetInt16ById.Request(id=8, data=1), 'motion_enable'):
            self.get_logger().error("Failed to enable motion. Aborting.")
            return
        if not self.call_service(self.set_mode_client, SetInt16.Request(data=0), 'set_mode'):
            self.get_logger().error("Failed to set mode. Aborting.")
            return
        if not self.call_service(self.set_state_client, SetInt16.Request(data=0), 'set_state'):
            self.get_logger().error("Failed to set state. Aborting.")
            return

        self.get_logger().info("Opening gripper and moving to start pose...")
        self.move_gripper(GRIPPER_OPEN_METERS)

        if not self.move_robot_line(SAFE_TRAVEL_POSE):
            self.get_logger().error("Failed to move to SAFE_TRAVEL_POSE. Aborting setup.")
            return

        if not self.move_robot_line(INSPECTION_POSE):
            self.get_logger().error("Failed to move to INSPECTION_POSE. Aborting setup.")
            return

        with self.state_lock:
            self.robot_is_moving = False
        self.get_logger().info("🤖 Robot setup complete. Listening for objects...")

        self.create_timer(1.0, self.execute_grasp_sequence)

def main(args=None):
    rclpy.init(args=args)
    node = FoundationPoseGraspBridge()
    
    ## --- FIX ---: Use a MultiThreadedExecutor to handle callbacks and futures safely.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the setup in a separate thread.
    # setup_thread = Thread(target=node.run_setup_and_main_loop, daemon=True)
    # setup_thread.start()

    try:
        # Spin the executor instead of the node directly.
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()