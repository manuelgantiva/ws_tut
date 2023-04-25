#include "rclcpp/rclcpp.hpp"
#include "ude_interface/srv/turn_camera.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>

using std::placeholders::_1;

const double time_default_ = 5.0;

class TurnCameraClientNode : public rclcpp::Node 
{
public:
    TurnCameraClientNode() : Node("turn_camera_client")
    {
        this-> declare_parameter("client_time", time_default_);
        double client_time_ = this-> get_parameter("client_time").as_double();
            
        timer_=this->create_wall_timer(std::chrono::milliseconds((int) (1000*client_time_)), 
                                        std::bind(&TurnCameraClientNode::callChangedAngle,this));
        client = this->create_client<ude_interface::srv::TurnCamera>("turn_camera");
        RCLCPP_INFO(this->get_logger(), "Turn Camera Client Node has been started.");
        
    }

    

private:
    void callChangedAngle(){
        double x = (randomDouble() * 10.0)-5.0;
        threads_.push_back(std::thread(std::bind(&TurnCameraClientNode::callTurnCameraService, this, x)));
    }

    void callTurnCameraService(float degree_turn)
    { 
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<ude_interface::srv::TurnCamera::Request>();
        request->degree_turn = degree_turn;

        client->async_send_request(request,std::bind(&TurnCameraClientNode::response_received_callback, this, _1));
    }

    void response_received_callback(rclcpp::Client<ude_interface::srv::TurnCamera>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            auto cv_ptr = cv_bridge::toCvCopy(response.second.get()->camera_image, "bgr8");
            auto image = cv_ptr->image;
            RCLCPP_INFO(this->get_logger(), "Degree turned is %lf", response.first->degree_turn);
            cv::imshow("Robot Camera Image", image);
            cv::waitKey(0);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
}

    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    rclcpp::Client<ude_interface::srv::TurnCamera>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurnCameraClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}