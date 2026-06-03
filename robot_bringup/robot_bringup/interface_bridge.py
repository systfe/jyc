import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


class InterfaceBridge(Node):
    def __init__(self):
        super().__init__('interface_bridge')

        cmd_input_topic = self.declare_parameter('cmd_input_topic', '/robot/cmd_vel').value
        cmd_output_topic = self.declare_parameter('cmd_output_topic', '/cmd_vel').value
        image_input_topic = self.declare_parameter('image_input_topic', '/omni_camera/image_raw').value
        image_output_topic = self.declare_parameter('image_output_topic', '/robot/image_raw').value
        odom_input_topic = self.declare_parameter('odom_input_topic', '/odom').value
        odom_output_topic = self.declare_parameter('odom_output_topic', '/robot/odom').value

        self.cmd_pub = self.create_publisher(Twist, cmd_output_topic, 10)
        self.image_pub = self.create_publisher(Image, image_output_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_output_topic, 10)

        self.cmd_sub = self.create_subscription(Twist, cmd_input_topic, self.cmd_callback, 10)
        self.image_sub = self.create_subscription(Image, image_input_topic, self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_input_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'cmd {cmd_input_topic} -> {cmd_output_topic}, '
            f'image {image_input_topic} -> {image_output_topic}, '
            f'odom {odom_input_topic} -> {odom_output_topic}'
        )

    def cmd_callback(self, msg):
        self.cmd_pub.publish(msg)

    def image_callback(self, msg):
        self.image_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
