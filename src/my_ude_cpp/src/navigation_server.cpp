#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ude_interface/action/navigate.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::Point;

typedef ude_interface::action::Navigate Navigate;
typedef rclcpp_action::ServerGoalHandle<Navigate> GoalHandle;

const float DIST_THRESHOLD = 0.125; 

class NavigationServerNode : public rclcpp::Node
{
public:
    NavigationServerNode() : Node("navigation_server") 
    {
        robot_position_ = Point();
        subscription_ = this->create_subscription<Point>("robot_position", 10, 
                std::bind(&NavigationServerNode::update_robot_position, this, std::placeholders::_1));
        action_server_= rclcpp_action::create_server<Navigate>(this ,"navigate",
                        std::bind(&NavigationServerNode::handle_goal, this ,  _1, _2),
                        std::bind(&NavigationServerNode::handle_cancel, this ,  _1),
                        std::bind(&NavigationServerNode::handle_accepted, this ,  _1));
    	RCLCPP_INFO(this->get_logger(), "Navigation Server Node has been started.");
    }

private:
    void update_robot_position(const Point & msg)
    {
      robot_position_ = msg;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const Navigate::Goal> goal)
    {   
        (void)uuid;
        (void)goal;
        //RCLCPP_INFO(get_logger(), "Received goal request with ID: %i", uuid);
        RCLCPP_INFO(get_logger(), "Received goal request with ID");
        /*float x = goal->goal_point.x;
        float y = goal->goal_point.y;
        float z = goal->goal_point.z;*/
        rclcpp_action::GoalResponse result = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        return result;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle)
    {
        (void)goal_handle;
        //RCLCPP_INFO(get_logger(), "Received cancel request for goal request with ID: %i", goal_handle->get_goal_id());
        RCLCPP_INFO(get_logger(), "Received cancel request for goal request with ID");
        rclcpp_action::CancelResponse result = rclcpp_action::CancelResponse::ACCEPT;
        return result;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle)
    {
        //RCLCPP_INFO(get_logger(), "Goal request with ID %i has been accepted.", goal_handle->get_goal_id());
        RCLCPP_INFO(get_logger(), "Goal request with ID has been accepted.");
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&NavigationServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto start_time = rclcpp::Clock().now();
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();
        feedback->distance_to_point = std::sqrt(
          std::pow(this->robot_position_.x - goal->goal_point.x, 2) + 
          std::pow(this->robot_position_.y - goal->goal_point.y, 2) + 
          std::pow(this->robot_position_.z - goal->goal_point.z, 2));
        auto robot_goal_point = Point();
        robot_goal_point = goal->goal_point;
        rclcpp::Rate loop_rate(1.0);
       
        if(this->robot_position_==robot_goal_point){
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
            result->elapsed_time = (rclcpp::Clock().now() - start_time).seconds();
            goal_handle->abort(result);
            return;
        }


        while(feedback->distance_to_point > DIST_THRESHOLD){
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->elapsed_time = (rclcpp::Clock().now() - start_time).seconds();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            feedback->distance_to_point = std::sqrt(
            std::pow(this->robot_position_.x - goal->goal_point.x, 2) + 
            std::pow(this->robot_position_.y - goal->goal_point.y, 2) + 
            std::pow(this->robot_position_.z - goal->goal_point.z, 2));

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
            loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        result->elapsed_time = (rclcpp::Clock().now() - start_time).seconds();
        goal_handle->succeed(result);
        
    }
    Point robot_position_;
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    rclcpp::Subscription<Point>::SharedPtr subscription_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}