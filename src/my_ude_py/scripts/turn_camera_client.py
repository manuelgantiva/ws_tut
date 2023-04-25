#!/usr/bin/env python3
import cv2
import rclpy
import random
from rclpy.node import Node
from ude_interface.srv import TurnCamera
from cv_bridge import CvBridge 
from functools import partial

TIME_DEFAULT_ = 5.0

class TurnCameraClientNode(Node): 
    def __init__(self):
        super().__init__("turn_camera_client") 
        
        self.declare_parameter("client_time", TIME_DEFAULT_)
        self.client_time_= self.get_parameter("client_time").value
        self.timer = self.create_timer(self.client_time_, self.call_change_angle)
        self.get_logger().info("Turn Camera Client Node has been started")
        

    def call_change_angle(self):
        x=random.randint(-100,100)/10
        self.call_turn_camera_server(x)


    def call_turn_camera_server(self, degree_turn):
        client = self.create_client(TurnCamera, "turn_camera")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to be up... ")
        
        request = TurnCamera.Request()
        request.degree_turn = degree_turn
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_turn_camera_resp, degree_turn=degree_turn))

    def callback_turn_camera_resp(self, future, degree_turn):
        try:
            response = future.result()
            self.get_logger().info("Degree turned is " + str(degree_turn))
            image = CvBridge().imgmsg_to_cv2(response.camera_image)
            cv2.imshow("Turn Camera Image", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurnCameraClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()