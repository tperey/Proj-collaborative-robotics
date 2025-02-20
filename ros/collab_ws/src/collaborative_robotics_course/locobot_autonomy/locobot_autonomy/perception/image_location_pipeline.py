#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import io
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from google.cloud import vision
from PIL import Image as PILImage
from geometry_msgs.msg import Pose, Twist



class VisionObjectDetector:
    def __init__(self):
        """Initialize the Google Cloud Vision client."""
        self.client = vision.ImageAnnotatorClient()

    def find_center(self, image_bytes, object_name):
        """
        Finds the center of an object (e.g., "apple") in the provided image bytes.

        :param image_bytes: The raw bytes of the image.
        :param object_name: The target object name to search for (case-insensitive).
        :return: Tuple (pixel_x, pixel_y) of the object's approximate center, or None if not found.
        """

        image = vision.Image(content=image_bytes)

        response = self.client.object_localization(image=image)

        objects = response.localized_object_annotations

        img = PILImage.open(io.BytesIO(image_bytes))
        width, height = img.size

        for obj in objects:
            if obj.name.lower() == object_name.lower():
                sides = obj.bounding_poly.normalized_vertices
                if len(sides) < 4:
                    return None
                x_center = (sides[0].x + sides[2].x) / 2 * width
                y_center = (sides[0].y + sides[2].y) / 2 * height
                return int(x_center), int(y_center)

        return None


class AppleOdomVLMPipeline(Node):
    def __init__(self):
        super().__init__('vlm_apple_odom_pipeline')
        self.bridge = CvBridge()
        self.detector = VisionObjectDetector()

        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.latest_depth_image = None

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.latest_position = None
        self.latest_velocity = None

        self.get_logger().info("VLM-based Apple Detection with Odometry Pipeline Started.")

    def image_callback(self, msg):
        """ Process RealSense camera images for apple detection using VLM (Google Cloud Vision). """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        _, img_encoded = cv2.imencode('.jpg', cv_image)
        image_bytes = img_encoded.tobytes()

        apple_center = self.detector.find_center(image_bytes, "apple")

        if apple_center:
            x_pixel, y_pixel = apple_center
            cv2.circle(cv_image, (x_pixel, y_pixel), 10, (0, 255, 0), 2)
            self.get_logger().info(f"Apple detected at pixel coordinates: ({x_pixel}, {y_pixel})")

            if self.latest_depth_image is not None:
                depth_value = self.latest_depth_image[y_pixel, x_pixel] * 0.001

                if self.latest_position:
                    px, py = self.latest_position
                    vx, vy = self.latest_velocity
                    self.get_logger().info(
                        f"Apple Depth: {depth_value:.2f}m | "
                        f"Robot Position: x={px:.2f}, y={py:.2f} | "
                        f"Velocity: vx={vx:.2f}, vy={vy:.2f}"
                    )

        cv2.imshow("Apple Detection", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        """ Store latest depth image for apple depth estimation. """
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def odom_callback(self, msg):
        """ Store latest odometry data. """
        position: Pose = msg.pose.pose
        velocity: Twist = msg.twist.twist

        self.latest_position = (position.position.x, position.position.y)

        self.latest_velocity = (velocity.linear.x, velocity.linear.y)


def main(args=None):
    rclpy.init(args=args)
    pipeline = AppleOdomVLMPipeline()
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
