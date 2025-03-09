import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)

        # OpenCV Video Capture
        self.cap = cv2.VideoCapture("udp://0.0.0.0:5005", cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video stream")

        self.bridge = CvBridge()
        
        # Timer for publishing images
        self.timer = self.create_timer(0.033, self.publish_image)  # 30 FPS

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture image")
            return
        
        frame_resized = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        # Convert frame to ROS2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert frame to compressed image
        _, compressed_frame = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 5])
        compressed_msg = CompressedImage()
        compressed_msg.header = img_msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(compressed_frame).tobytes()

        # Publish messages
        self.image_pub.publish(img_msg)
        self.compressed_pub.publish(compressed_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
