#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

class TaskPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr test_node_;
};

TEST_F(TaskPublisher, test_num_publishers) {
  test_node_ = rclcpp::Node::make_shared("test_pub");
  auto test_pub =
      test_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto pub_count = test_node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(pub_count));
}