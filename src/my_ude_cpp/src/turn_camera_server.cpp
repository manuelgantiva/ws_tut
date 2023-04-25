#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ude_interface/srv/turn_camera.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>

using namespace std::chrono_literals;

const float angle_initial_ = 0.0;
const double freq_default_ = 1.0;

using std::placeholders::_1;
using std::placeholders::_2;

class TurnCameraServerNode : public rclcpp::Node 
{
public:
    TurnCameraServerNode(std::string exe_dir) : Node("turn_camera_server")
    {
        ws_dir_ = get_ws_dir(exe_dir);
        this->declare_parameter("angle_initial",angle_initial_);
        angle_= this-> get_parameter("angle_initial").as_double();

        this-> declare_parameter("publish_frecuency", freq_default_);
        double publish_frecuency = this-> get_parameter("publish_frecuency").as_double();

        publisher_=this->create_publisher<std_msgs::msg::Float32>("angle",10);
        timer_=this->create_wall_timer(std::chrono::milliseconds((int) (1000/publish_frecuency)), 
                                        std::bind(&TurnCameraServerNode::callPublishAngle,this));
        
        server_ = this-> create_service<ude_interface::srv::TurnCamera>("turn_camera",
                std::bind(&TurnCameraServerNode::callbackTurnCamera, this, _1, _2));
    	RCLCPP_INFO(this->get_logger(), "Turn Camera Server Node has been started.");
    }

private:

    void callPublishAngle(){
        auto msg=std_msgs::msg::Float32();
        msg.data= angle_;
        publisher_->publish(msg);
    }

    void callbackTurnCamera(const ude_interface::srv::TurnCamera::Request::SharedPtr request,
                            const ude_interface::srv::TurnCamera::Response::SharedPtr response)
    {
        angle_=angle_-request->degree_turn;
        if(angle_<-30.0){
            angle_=-30.0;
        }else if (angle_>30.0)
        {
            angle_=30.0;
        }

        float closest_num = available_angles_[0];
		float smallest_angle = std::abs(angle_ - available_angles_[0]);
        float angle_diff;
        
        for (int i=1; i<5; i++) //Based on size of available_angles_ array
        {
            angle_diff = std::abs(angle_ - available_angles_[i]);
            if (angle_diff < smallest_angle)
            {
                smallest_angle = angle_diff;
                closest_num = available_angles_[i]; 
            }
        }
        std::string image_path = ws_dir_ + "src/my_ude_cpp/images/" + std::to_string((int)closest_num) + ".png";
        RCLCPP_INFO(this->get_logger(), "the path is %s", image_path.c_str());

        auto image = cv::imread(image_path);
        auto image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

        response->camera_image = *image_ptr;
    }

    std::string get_ws_dir(std::string exe_dir)
        {
            std::string::size_type substr_index = exe_dir.find("install");
            return exe_dir.substr(0, substr_index);
        }
    float angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<ude_interface::srv::TurnCamera>::SharedPtr server_;
    const float available_angles_ [5] {-30, -15, 0, 15, 30};
    std::string ws_dir_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurnCameraServerNode>(argv[0]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}