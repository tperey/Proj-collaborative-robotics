#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change the message type if needed
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class ParamPublisher(Node):
    def __init__(self):
        super().__init__("param_publisher")

        # Declare and get the parameter
        self.declare_parameter("robot_description_semantic", "default_value")  # Default value if not set
        param_value = self.get_parameter("robot_description_semantic").value

        # Set QoS profile to Transient Local
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create a publisher with the corrected QoS
        self.publisher_ = self.create_publisher(String, "/robot_description_semantic", qos_profile)

        # # Publish the parameter value periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.msg = String()
        self.msg.data = str(param_value)

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Published data!")

def main(args=None):
    rclpy.init(args=args)
    node = ParamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
