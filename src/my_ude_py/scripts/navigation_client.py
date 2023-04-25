#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from functools import partial

from ude_interface.action import Navigate
from std_msgs.msg import Empty


class NavigationClientNode(Node): 
    def __init__(self):
        super().__init__("navigation_client") 
        self.subscriber_ = self.create_subscription(Empty, "cancel_navigate",self.callback_cancel_goal,1)
        self.call_navigate_server(2.0, 3.0, 0.0)
        self.get_logger().info("Navigation Client Node has been started")

    def callback_cancel_goal(self, msg):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def call_navigate_server(self, x, y, z):
        self.action_client_=ActionClient(self, Navigate, 'navigate')
        while not self.action_client_.wait_for_server(1.0):
            self.get_logger().warn("Waiting for server Action... ")

        goal_msg = Navigate.Goal()
        goal_msg.goal_point.x = float(x)
        goal_msg.goal_point.y = float(y)
        goal_msg.goal_point.z = float(z)

        self.get_logger().info("Sending goal")

        send_goal_future = self.action_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(partial(self.goal_response_callback, x=x, y=y, z=z))

    def goal_response_callback(self, future, x, y, z):
        goal_handle = future.result()
        status_=goal_handle.status
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server")
            return
        self.get_logger().info("Goal accepted by server, waiting for result...")                    
        self.get_logger().info("Status is: " + str(status_) + ", X is : " + str(x) +
                               ", Y is : " + str(y) + ", Z is : " + str(z))
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(partial(self.get_result_callback, x=x, y=y, z=z))

    def get_result_callback(self, future, x, y, z):
        try:
            status = future.result().status
            result = future.result().result
            if(status==0):
                self.get_logger().warn("Unknown result code")  
            elif(status==5):
                self.get_logger().warn("Goal was canceled")  
            elif(status==4):
                self.get_logger().info("Goal was succeeded")  
            elif(status==6):
                self.get_logger().warn("Goal was aborted")  

            self.get_logger().info("Status is: " + str(status) + ", X is : " + str(x) +
                               ", Y is : " + str(y) + ", Z is : " + str(z))
            self.get_logger().info('Received result:  {0}'.format(result.elapsed_time))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance_to_point))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()