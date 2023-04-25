#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/msg/empty.hpp"

#include "ude_interface/action/navigate.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::Point;

typedef ude_interface::action::Navigate Navigate;
typedef rclcpp_action::ClientGoalHandle<Navigate> GoalHandle;
typedef rclcpp_action::Client<Navigate>::CancelResponse CancelResponse; 

class NavigationClientNode : public rclcpp::Node 
{
public:
    NavigationClientNode() : Node("navigation_client") 
    {
        subscriber_ = this-> create_subscription<example_interfaces::msg::Empty>("cancel_navigate",1,
                std::bind(&NavigationClientNode::callback_cancel_goal, this, std::placeholders::_1));
        action_client_ =  rclcpp_action::create_client<Navigate>(this, "navigate");
        call_navigate_server(3.0, 5.0, 0.0);
    	RCLCPP_INFO(this->get_logger(), "Navigation Client Node has been started.");
    }

private:
    void callback_cancel_goal(const example_interfaces::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Canceling goal");
        action_client_->async_cancel_goal(this->goal_handle_,std::bind(&NavigationClientNode::cancel_done, this, _1));
    }

    void cancel_done(const CancelResponse::SharedPtr & cancel_response){
        switch (cancel_response->return_code) {
        case CancelResponse::ERROR_NONE:
            RCLCPP_INFO(this->get_logger(), "Goal successfully canceled");
            break;
        case CancelResponse::ERROR_REJECTED:
            RCLCPP_WARN(this->get_logger(), "Goal reject to cancel");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Goal failed to cancel");
            break;
        }
    }

    void call_navigate_server(float x, float y, float z)
    {
        
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
        }
        auto goal_msg = Navigate::Goal();
        goal_msg.goal_point.x = x;
        goal_msg.goal_point.y = y;
        goal_msg.goal_point.z = z;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigationClientNode::goal_response_callback, this, _1, 
            x, y, z);
        send_goal_options.feedback_callback = std::bind(&NavigationClientNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&NavigationClientNode::result_callback, this, _1, x, y, z);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandle::SharedPtr & goal_handle, float x, float y, float z)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            auto status_= goal_handle->get_status();
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
            RCLCPP_INFO(this->get_logger(), "Status is: %d, X is : %f, Y is :  %f, Z is :  %f", status_ ,x, y, z);
            this->goal_handle_=goal_handle;
        }
    }

    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const Navigate::Feedback> feedback)
    {
        float distance_to_point = feedback->distance_to_point;
        RCLCPP_INFO(this->get_logger(), "Received feedback: %f", distance_to_point);
    }

    void result_callback(const GoalHandle::WrappedResult & result, float x, float y, float z)
    {
        auto status_=result.code;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code");  
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Status is: %d, X is : %f, Y is :  %f, Z is :  %f", int(status_) ,x, y, z);
        int elapsed_time_f = result.result->elapsed_time;
        RCLCPP_INFO(this->get_logger(), "Received result: %d", elapsed_time_f);
    }


    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr subscriber_;
    GoalHandle::SharedPtr goal_handle_;
    rclcpp_action::Client<Navigate>::SharedPtr action_client_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationClientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}