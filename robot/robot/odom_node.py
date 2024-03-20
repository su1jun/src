import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self, qos=rclpy.qos.qos_profile_sensor_data)
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

    def cmd_vel_callback(self, msg):
        # just cmd_vel
        self.odom.twist.twist = msg
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.odom)

        # TF transform
        t = TransformStamped()
        t.header.stamp = self.odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom.pose.pose.position.x
        t.transform.translation.y = self.odom.pose.pose.position.y
        t.transform.translation.z = self.odom.pose.pose.position.z
        t.transform.rotation = self.odom.pose.pose.orientation
        self.odom_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()