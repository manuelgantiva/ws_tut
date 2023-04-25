#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

const double wheelradio_default = 1.0;

class SpeedometerNode : public rclcpp::Node 
{
public:
    SpeedometerNode() : Node("speedometer")
    {
        this->declare_parameter("wheel_radio",wheelradio_default);
        publisher_=this->create_publisher<std_msgs::msg::Float32>("velocity",10);
        subscriber_ = this-> create_subscription<std_msgs::msg::Float32>("rpm",10,
                std::bind(&SpeedometerNode::callRpm, this, _1));
        RCLCPP_INFO(this->get_logger(), "Speedometer has been started.");
    }

private:

    void callRpm(const std_msgs::msg::Float32::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Los RPM son de: %f", msg ->data);
        auto message=std_msgs::msg::Float32();
        wheelradio_=this->get_parameter("wheel_radio").as_double();
        message.data= msg->data*wheelradio_;
        publisher_->publish(message);
    }

    float wheelradio_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedometerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}