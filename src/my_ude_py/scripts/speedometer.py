#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

WHEELRADIO_DEFFAULT_=1.0;

class SpeedometerNode(Node): 
    def __init__(self):
        super().__init__("speedometer") 
        self.declare_parameter("wheel_radio",WHEELRADIO_DEFFAULT_)
        self.suscreiber_=self.create_subscription(Float32,"rpm",self.call_rpm,10)
        self.publisher_=self.create_publisher(Float32,"velocity",10)
        self.get_logger().info("Speedometer has been started")

    def call_rpm(self, msg):
        self.get_logger().info("Los RPM son de :" + str(msg.data))
        message=Float32()
        self.wheelradio_= self.get_parameter("wheel_radio").get_parameter_value().double_value
        message.data=self.wheelradio_*msg.data
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedometerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()