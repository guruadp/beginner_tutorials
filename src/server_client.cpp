#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<iostream>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/update_string.hpp"

using namespace std::chrono_literals;

class ServerClient : public rclcpp::Node{
    public:
        ServerClient() : Node("server_client"){
            client = this->create_client<beginner_tutorials::srv::UpdateString>("service_node"); 
        }
        auto getRequest(char **argv){
            auto request = std::make_shared<beginner_tutorials::srv::UpdateString::Request>();
            request->ipString = argv[1]; 
            return request;
        }

        rclcpp::Client<beginner_tutorials::srv::UpdateString>::SharedPtr client;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    std::shared_ptr<ServerClient> SClient = std::make_shared<ServerClient>();
    while (!SClient->client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exiting...");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the service...");
    }
    auto request = SClient->getRequest(argv);
    auto result = SClient->client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(SClient, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "update string '%s'",result.get()->opString.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service update_string");
    }

    rclcpp::shutdown();
  return 0;
}