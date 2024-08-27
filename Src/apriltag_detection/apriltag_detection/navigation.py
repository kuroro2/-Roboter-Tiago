import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math

class NavigateToAprilTag(Node):

    def __init__(self):
        super().__init__('navigate_to_apriltag')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',  # Topic from the AprilTag detection node
            self.tag_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            self.get_logger().info('No AprilTags detected')
            return

        tag = msg.detections[0]

        # Log the tag object to inspect its structure
        self.get_logger().info(f'Received tag detection: {tag}')
        
        try:
            tag_center = tag.centre
        except AttributeError:
            self.get_logger().error(f'Tag detection does not have the expected centre attribute. Full tag content: {tag}')
            return
        
        self.navigate_to_tag(tag_center)

    def navigate_to_tag(self, tag_center):
        if self.current_pose is None:
            return

        # Assuming the tag center gives us the 2D coordinates to navigate towards
        dx = tag_center.x - self.current_pose.position.x
        dy = tag_center.y - self.current_pose.position.y

        # Calculate the distance to the tag
        distance = math.sqrt(dx * dx + dy * dy)

        # Log the calculated distance and direction
        self.get_logger().info(f'Distance to tag: {distance}')

        # Threshold for the distance to the tag
        if distance < 0.5:
            self.get_logger().info('Reached the AprilTag')
            return

        # Calculate the required angular direction
        angle_to_tag = math.atan2(dy, dx)

        # Create and publish control commands
        twist = Twist()
        twist.linear.x = 1 if distance > 0.5 else 0.0
        twist.angular.z = angle_to_tag
        self.publisher.publish(twist)
        
        # Log the Twist message
        self.get_logger().info(f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToAprilTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()