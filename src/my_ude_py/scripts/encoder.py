#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

RPM_DEFAULT_=5.0


class EncoderNode(Node): 
    def __init__(self):
        super().__init__("encoder") 
        self.declare_parameter("rpm_value", RPM_DEFAULT_)
        self.publisher_ = self.create_publisher(Float32, "rpm", 10)
        self.timer = self.create_timer(0.5, self.call_publish_rpm)
        self.get_logger().info("Encoder has been started")

    def call_publish_rpm(self):
         msg=Float32()
         self.rpm_ = self.get_parameter("rpm_value").get_parameter_value().double_value
         msg.data= self.rpm_
         self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()