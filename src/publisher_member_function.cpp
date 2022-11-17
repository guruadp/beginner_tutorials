/**
 * @file publisher_member_function.cpp
 * @author Guru Nandhan A D P(guruadp@umd.edu)
 * @brief A publisher that publishes a string and gets update string from a server
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/srv/update_string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief MinimalPublisher class has the publisher node and server that updates string
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    try{

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
      param_desc.description = "Set callback frequency.";
      this->declare_parameter("freq", 2.0, param_desc);
      auto param = this->get_parameter("freq");
      auto freq = param.get_parameter_value().get<std::float_t>();
      m_param_subscriber_ =
          std::make_shared<rclcpp::ParameterEventHandler>(this);
      auto paramCallbackPtr =
          std::bind(&MinimalPublisher::param_callback, this, _1);
      m_paramHandle_ =
          m_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      auto period = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
      timer_ = this->create_wall_timer(
          period, std::bind(&MinimalPublisher::timer_callback, this));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "publisher is initialized");
      
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing server"); 
      server = this->create_service<cpp_pubsub::srv::UpdateString>("service_node", std::bind(&MinimalPublisher::update_string,this,std::placeholders::_1,std::placeholders::_2));   
      
    }
    catch(...){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error during initialization !");
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work !");
      RCLCPP_WARN_STREAM(this->get_logger(), "Some error occured !");
    }
  }
  /**
   * @brief this function gets input and responds
   * 
   * @param request 
   * @param response 
   */
  void update_string(const std::shared_ptr<cpp_pubsub::srv::UpdateString::Request> request,   
          std::shared_ptr<cpp_pubsub::srv::UpdateString::Response>       response) {
  response->output = request->input;

  resp_message = response->output;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ninput: '%s'",request->input.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'",response->output.c_str());}

 private:
  PARAMETER_EVENT m_param_subscriber_;
  PARAMETER_HANDLE m_paramHandle_;

  /**
   * @brief this function publishes the string
   * 
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = resp_message;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  /**
   * @brief this function controls the frequency of printing
   * 
   * @param param 
   */
  void param_callback(const rclcpp::Parameter& param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());

    auto period = std::chrono::milliseconds(static_cast<int>
    (1000 / param.as_double()));
    auto topicCallbackPtr = std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(period,
                                     topicCallbackPtr);  // no memory leak here
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::UpdateString>::SharedPtr server; 
  std::string resp_message = "Hello";
  size_t count_;
};
/**
 * @brief this is the main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
