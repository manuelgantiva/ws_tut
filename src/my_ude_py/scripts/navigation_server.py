#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from ude_interface.action import Navigate

DISTANCE_THRESHOLD = 0.125

class NavigationServerNode(Node): 
    def __init__(self):
        super().__init__("navigation_server") 
        self.subscriber_  = self.create_subscription(Point, "robot_position", self.update_robot_position, 1)
        self.robot_current_position = None 
        self.action_server_= ActionServer(self, Navigate, 'navigate', 
                                        execute_callback=self.execute_callback,
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback)
        self.get_logger().info("Navigation Server Node has been started")

    def update_robot_position(self, point):
        self.robot_current_position = [point.x, point.y, point.z]

    def handle_accepted_callback(self, goal_handle):
        """ Maneja el goal aceptado """
        goal_id = goal_handle.goal_id
        self._logger.info("Goal request with ID: " + str(goal_handle.goal_id) + " has been accepted.")

    def goal_callback(self, goal_request):
        """ Maneja el goal enviado por el cliente """
        self.get_logger().info("Received goal request with ID: " + str(goal_request))
        x = goal_request.goal_point.x
        y = goal_request.goal_point.y
        z = goal_request.goal_point.z       
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request for goal request with ID: " + str(goal_handle.goal_id))
        return CancelResponse.ACCEPT

    
    def execute_callback(self, goal_handle):
        self.get_logger().info("Goal request with ID: " + str(goal_handle.goal_id) + " has been accepted.")    
        self.get_logger().info("Executing goal")    
        start_time = self.get_clock().now().to_msg().sec 
        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()
        robot_goal_point = [goal_handle.request.goal_point.x, 
							goal_handle.request.goal_point.y,
							goal_handle.request.goal_point.z]
        
        
        self.get_logger().info("Goal Point: " + str(robot_goal_point))

        while self.robot_current_position == None:
            self.get_logger().warn("Robot Point Not Detected")
            time.sleep(3.0)

        distance_to_goal = math.dist(self.robot_current_position, robot_goal_point)
        feedback_msg = Navigate.Feedback()
        
        if(self.robot_current_position==robot_goal_point):
            self.get_logger().info("Goal aborted")
            goal_handle.abort()
            result.elapsed_time = float(self.get_clock().now().to_msg().sec - start_time)
            return result
        
        while(distance_to_goal > DISTANCE_THRESHOLD):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                result.elapsed_time = float(self.get_clock().now().to_msg().sec - start_time)
                goal_handle.canceled()
                return result
            distance_to_goal = math.dist(self.robot_current_position, robot_goal_point)
            feedback_msg.distance_to_point = distance_to_goal
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Publish feedback")
            time.sleep(1.0)

        self.get_logger().info("Goal succeeded")
        goal_handle.succeed()
        result.elapsed_time = float(self.get_clock().now().to_msg().sec - start_time)
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigationServerNode() 
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()