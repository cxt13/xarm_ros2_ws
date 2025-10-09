import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

rclpy.init()
node = Node('tf_test_node')
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

pose = PoseStamped()
pose.header.frame_id = 'camera_color_optical_frame'
pose.header.stamp = node.get_clock().now().to_msg()
pose.pose.position.x = 0.5
pose.pose.position.y = 0.0
pose.pose.position.z = 0.2
pose.pose.orientation.w = 1.0

time.sleep(0.5)
try:
    trans = tf_buffer.lookup_transform('link_base', 'camera_color_optical_frame',
                                       rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    p = do_transform_pose(pose, trans)
    print('Transformed pose in link_base:', p)
except Exception as e:
    node.get_logger().error('TF lookup failed: %s' % e)
rclpy.shutdown()
