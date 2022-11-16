// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    try{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing server"); 
      server = this->create_service<beginner_tutorials::srv::UpdateString>("service_node", std::bind(&Publisher::UpdateString,this,std::placeholders::_1,std::placeholders::_2));   
      
    }
    catch(...){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error during initialization!!");
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }

  }

  void updateString(const std::shared_ptr<beginner_tutorials::srv::UpdateString::Request> request,   
          std::shared_ptr<beginner_tutorials::srv::UpdateString::Response>       response) {
  response->opString = request->ipString 

  server_resp_message = response->opString;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ninput: '%s'",request->ipString.c_str()); //+
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'",response->opString.c_str());}

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_resp_message;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
