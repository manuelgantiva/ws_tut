#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from ude_interface.srv import TurnCamera
from cv_bridge import CvBridge 
from std_msgs.msg import Float32

ANGLE_INITIAL_ = 0.0
FREQUENCY_DEFAULT_ = 1.0


class TurnCameraServerNode(Node): 
    def __init__(self):
        super().__init__("turn_camera_server") 

        self.declare_parameter("angle_initial", ANGLE_INITIAL_)
        self.declare_parameter("publish_frecuency", FREQUENCY_DEFAULT_)

        self.angle_= self.get_parameter("angle_initial").value
        self.publish_frecuency_=self.get_parameter("publish_frecuency").value

        self.available_angles = [-30, -15, 0, 15, 30]

        self.publisher_ = self.create_publisher(Float32, "angle", 10)
        self.timer = self.create_timer(1/self.publish_frecuency_, self.call_publish_angle)

        self.server_=self.create_service(TurnCamera, "turn_camera", self.callback_turn_camare)
        self.get_logger().info("Turn Camera Server Node has been started")

    def call_publish_angle(self):
         msg=Float32()
         msg.data= self.angle_
         self.publisher_.publish(msg)
    
    def callback_turn_camare(self, request, response):
        self.angle_=self.angle_-request.degree_turn
        if(self.angle_>30.0):
            self.angle_=30.0
        elif(self.angle_<-30.0):
            self.angle_=-30.0
        image = self.get_image(self.angle_)
        image_msg = CvBridge().cv2_to_imgmsg(image)
        response.camera_image = image_msg 
        return response 


    def get_image(self, angle):
        closest_angle = min(self.available_angles, key=lambda x:abs(x-angle))
        return self.read_in_image_by_filename(str(closest_angle) + ".png")

    def read_in_image_by_filename(self, file_name):
        dir_name = os.path.dirname(__file__)
        install_dir_index = dir_name.index("install/")
        file_location = dir_name[0:install_dir_index] + "src/my_ude_py/images/" + file_name 
        image = cv2.imread(file_location)
        return image 


def main(args=None):
    rclpy.init(args=args)
    node = TurnCameraServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()