#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <chrono>

using namespace std::chrono_literals;

const double rpm_default_ = 5.0;


class EncoderNode : public rclcpp::Node 
{
public:
    EncoderNode() : Node("encoder") 
    {
        this->declare_parameter("rpm_value",rpm_default_);
        publisher_=this->create_publisher<std_msgs::msg::Float32>("rpm",10);
        timer_=this->create_wall_timer(1s, std::bind(&EncoderNode::callPublishRpm,this));
    	RCLCPP_INFO(this->get_logger(), "Encoder has been started.");
    }

private:

    void callPublishRpm(){
        auto msg=std_msgs::msg::Float32();
        rpm= this-> get_parameter("rpm_value").as_double();
        msg.data= rpm;
        publisher_->publish(msg);
    }

    float rpm;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}