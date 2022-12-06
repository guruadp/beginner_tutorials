// MIT License

// Copyright (c) 2022 Guru Nandhan A D P

// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
// DEALINGS IN THE SOFTWARE.

/**
 * @file publisher_member_function.cpp
 * @author Guru Nandhan A D P(guruadp@umd.edu)
 * @brief A publisher that publishes a string and gets update string from a
 * server
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

#include "cpp_pubsub/srv/update_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief MinimalPublisher class has the publisher node and server that updates
 * string
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    try {
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
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
      auto period = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
      timer_ = this->create_wall_timer(
          period, std::bind(&MinimalPublisher::timer_callback, this));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "publisher is initialized");

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing server");
      server = this->create_service<cpp_pubsub::srv::UpdateString>(
          "service_node",
          std::bind(&MinimalPublisher::update_string, this,
                    std::placeholders::_1, std::placeholders::_2));
    //tf
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();

    } catch (...) {
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
  void update_string(
      const std::shared_ptr<cpp_pubsub::srv::UpdateString::Request> request,
      std::shared_ptr<cpp_pubsub::srv::UpdateString::Response> response) {
    response->output = request->input;

    resp_message = response->output;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ninput: '%s'",
                request->input.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: '%s'",
                response->output.c_str());
  }

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
  void param_callback(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());

    auto period =
        std::chrono::milliseconds(static_cast<int>(1000 / param.as_double()));
    auto topicCallbackPtr = std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(period, topicCallbackPtr);
  }

  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "child";

    t.transform.translation.x = 90;
    t.transform.translation.y = 90;
    t.transform.translation.z = 90;
    tf2::Quaternion q;
    q.setRPY(
      90,
      90,
      90);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::UpdateString>::SharedPtr server;
  std::string resp_message = "Hello";
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
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
