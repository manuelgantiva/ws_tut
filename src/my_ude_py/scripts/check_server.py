#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ude_interface.srv import OddEvenCheck


class CheckServerNode(Node): 
    def __init__(self):
        super().__init__("check_server") 
        self.srv = self.create_service(OddEvenCheck, 'odd_even_check', self.determine_odd_even)
        self.get_logger().info("Check server has been started")

    def determine_odd_even(self, request, response): 
        if request.number % 2 == 0:
            response.decision = "Even"
        elif request.number % 2 == 1:
            response.decision = "Odd"
        else:
            response.decision = "Error"
        self.get_logger().info("the checked number is " + str(request.number) + " and the answer is " + response.decision)
        return response 


def main(args=None):
    rclpy.init(args=args)
    node = CheckServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()