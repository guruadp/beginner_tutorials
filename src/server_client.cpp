/**
 * @file server_client.cpp
 * @author Guru Nandhan A D P(guruadp@umd.edu)
 * @brief Server that updates the string in the publisher node
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "cpp_pubsub/srv/update_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief Initializes node and get string request
 *
 */
class ServerClient : public rclcpp::Node {
 public:
  ServerClient() : Node("server_client") {
    client = this->create_client<cpp_pubsub::srv::UpdateString>("service_node");
  }
  auto getRequest(char **argv) {
    auto request = std::make_shared<cpp_pubsub::srv::UpdateString::Request>();
    request->input = argv[1];
    return request;
  }

  rclcpp::Client<cpp_pubsub::srv::UpdateString>::SharedPtr client;
};

/**
 * @brief this is the main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

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
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "update string '%s'",
                result.get()->output.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service update_string");
  }

  rclcpp::shutdown();
  return 0;
}