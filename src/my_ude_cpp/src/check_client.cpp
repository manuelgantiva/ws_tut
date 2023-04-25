#include "rclcpp/rclcpp.hpp"
#include "ude_interface/srv/odd_even_check.hpp"

#include <iostream>

class CheckClientNode : public rclcpp::Node 
{
public:
    CheckClientNode() : Node("check_client")
    {
        threads_.push_back(std::thread(std::bind(&CheckClientNode::callOddEvenCheckService, this)));
    	RCLCPP_INFO(this->get_logger(), "Node has been started.");
    }

private:

    void callOddEvenCheckService()
    {
        auto client = this->create_client<ude_interface::srv::OddEvenCheck>("odd_even_check");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<ude_interface::srv::OddEvenCheck::Request>();
        std::cout << "Please type a number to check if it is Odd or Even: ";
	    std::cin >> request->number;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "the checked number is %d and the answer is %s", int(request->number), response->decision.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    std::vector<std::thread> threads_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckClientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}