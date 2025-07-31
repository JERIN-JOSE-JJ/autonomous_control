import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller_node')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.ultra_sub = self.create_subscription(Range, '/ultrasonic', self.ultrasonic_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/autonomous_cmd_vel', 10)

        self.min_lidar_dist = float('inf')
        self.ultrasonic_dist = float('inf')

    def lidar_callback(self, msg):
        self.min_lidar_dist = min(msg.ranges)
        self.decide_motion()

    def ultrasonic_callback(self, msg):
        self.ultrasonic_dist = msg.range
        self.decide_motion()

    def decide_motion(self):
        cmd = Twist()
        if self.min_lidar_dist < 0.5 or self.ultrasonic_dist < 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate to avoid obstacle
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
