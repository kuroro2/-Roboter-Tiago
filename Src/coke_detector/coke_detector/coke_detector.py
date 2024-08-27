import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import onnxruntime as ort
import numpy as np
import os
import time
import sensor_msgs_py.point_cloud2 as pc2  # Updated import path

class CokeDetector(Node):
    def __init__(self):
        super().__init__('coke_detector')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10)
        self.depth_subscription = self.create_subscription(
            PointCloud2,
            '/head_front_camera/depth_registered/points',
            self.depth_callback,
            10)
        self.bridge = CvBridge()
        self.model_path = '/home/ayman/runs/detect/train23/weights/best.onnx'
        self.session = ort.InferenceSession(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.publisher_ = self.create_publisher(PointStamped, 'coke_coordinates', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.get_logger().info('Coke Detector Node has been started')
        os.makedirs('detected_images', exist_ok=True)
        self.detections = []
        self.point_cloud = None

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.resize(cv_image, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (2, 0, 1)).astype(np.float32)
        img = np.expand_dims(img, axis=0)
        img /= 255.0

        ort_inputs = {self.input_name: img}
        ort_outs = self.session.run([self.output_name], ort_inputs)
        self.detections = ort_outs[0][0]

        self.get_logger().info(f'Detections shape: {self.detections.shape}')

        h, w, _ = cv_image.shape
        for detection in self.detections:
            x_center, y_center, width, height, conf = detection[:5]
            if conf > 0.2:
                x1 = int((x_center - width / 2) * w)
                y1 = int((y_center - height / 2) * h)
                x2 = int((x_center + width / 2) * w)
                y2 = int((y_center + height / 2) * h)

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f'Coke Can: {conf:.2f}'
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                self.get_logger().info(f'Coke can detected with confidence {conf:.2f}')

                self.publish_coordinates(x_center, y_center)

        filename = f'detected_images/{int(time.time())}.jpg'
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(filename, cv_image_bgr)
        self.get_logger().info(f'Image saved as {filename}')

    def depth_callback(self, msg):
        self.get_logger().info('Received point cloud')
        self.point_cloud = msg

    def publish_coordinates(self, x_center, y_center):
        if self.point_cloud is not None:
            x = int(x_center * self.point_cloud.width)
            y = int(y_center * self.point_cloud.height)

            # Ensure x and y are within bounds
            if x >= self.point_cloud.width or y >= self.point_cloud.height:
                self.get_logger().warn(f'Calculated x, y out of bounds: ({x}, {y})')
                return

            points = pc2.read_points(self.point_cloud, skip_nans=True, field_names=("x", "y", "z"))
            points = np.array(list(points))
            index = y * self.point_cloud.width + x

            if index >= points.shape[0]:
                self.get_logger().warn(f'Index out of bounds: {index}')
                return

            point = points[index]
            point_msg = PointStamped()
            point_msg.header.frame_id = 'head_front_camera_rgb_opical_frame'
            point_msg.point.x = point[0]
            point_msg.point.y = point[1]
            point_msg.point.z = point[2]
            self.publisher_.publish(point_msg)
            self.get_logger().info(f'Published coke coordinates: {point_msg.point.x}, {point_msg.point.y}, {point_msg.point.z}')

            # Publish a marker for visualization
            marker = Marker()
            marker.header.frame_id = 'head_front_camera_rgb_optical_frame'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'coke_detector'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_publisher.publish(marker)
            self.get_logger().info('Published visualization marker')

def main(args=None):
    rclpy.init(args=args)
    coke_detector = CokeDetector()
    rclpy.spin(coke_detector)
    coke_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

