#include "rclcpp/rclcpp.hpp"
#include "ude_interface/srv/odd_even_check.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class CheckServerNode : public rclcpp::Node 
{
public:
    CheckServerNode() : Node("check_server") 
    {
        service_server_ = this->create_service<ude_interface::srv::OddEvenCheck>("odd_even_check", 
                std::bind(&CheckServerNode::check_num_odd_even, this, _1, _2));
    	RCLCPP_INFO(this->get_logger(), "Check Server has been started.");
    }

private:
    void check_num_odd_even(const ude_interface::srv::OddEvenCheck::Request::SharedPtr request,
                            ude_interface::srv::OddEvenCheck::Response::SharedPtr response)
        {
            int remainder = std::abs(request->number % 2);
            switch(remainder) {
                case 0 :
                    response->decision = "Even";
                    break;
                case 1 :
                    response->decision = "Odd";
            }
            RCLCPP_INFO(this->get_logger(), "the checked number is %d and the answer is %s", int(request->number), response->decision.c_str());
        } 
    
    rclcpp::Service<ude_interface::srv::OddEvenCheck>::SharedPtr service_server_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}