// Copyright 2026 Meet Gandhi
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

#include "shmem_buffer_backend_test/shmem_image_publisher_node.hpp"

#include <chrono>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "shmem_buffer/shmem_buffer_api.hpp"
#include "shmem_buffer_backend_test/test_pattern_utils.hpp"

namespace shmem_buffer_backend_test
{

ShmemImagePublisherNode::ShmemImagePublisherNode(const rclcpp::NodeOptions & options)
: Node("shmem_image_publisher", options)
{
  topic_name_ = declare_parameter<std::string>("topic", "test_shmem_image");
  publish_rate_hz_ = declare_parameter<double>("rate", 10.0);
  image_width_ = declare_parameter<int>("width", 640);
  image_height_ = declare_parameter<int>("height", 480);
  encoding_ = declare_parameter<std::string>("encoding", "mono8");
  const int default_history = std::max(80, static_cast<int>(publish_rate_hz_ * 2.5));
  history_depth_ = declare_parameter<int>("history_depth", default_history);
  max_messages_ = declare_parameter<int>("max_messages", 0);
  wait_for_subscriber_ = declare_parameter<bool>("wait_for_subscriber", false);
  auto_exit_ = declare_parameter<bool>("auto_exit", false);
  use_shm_ = declare_parameter<bool>("use_shm", true);

  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, history_depth_);
  count_publisher_ = create_publisher<std_msgs::msg::UInt32>(topic_name_ + "/pub_count", 10);

  const auto timer_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&ShmemImagePublisherNode::publish_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "ShmemImagePublisherNode initialized: topic='%s', rate=%.1f Hz, res=%dx%d, history_depth=%d",
    topic_name_.c_str(), publish_rate_hz_, image_width_, image_height_, history_depth_);
}

void ShmemImagePublisherNode::publish_timer_callback()
{
  if (max_messages_ > 0 && static_cast<int>(sequence_) >= max_messages_) {
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Finished publishing %zu messages. Stopping.", sequence_);
    if (auto_exit_ && !shutdown_timer_) {
      shutdown_timer_ = create_wall_timer(
        std::chrono::milliseconds(2000),
        [this]() {
          shutdown_timer_->cancel();
          RCLCPP_INFO(get_logger(), "Publisher auto-exit triggered.");
          rclcpp::shutdown();
        });
    }
    return;
  }

  if (wait_for_subscriber_ && publisher_->get_subscription_count() == 0u) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for subscriber on '%s'...", topic_name_.c_str());
    return;
  }

  const std::size_t seq = sequence_ + 1u;
  const std::size_t payload_size = static_cast<std::size_t>(image_width_) * image_height_;

  sensor_msgs::msg::Image msg;
  msg.header.stamp = now();
  msg.header.frame_id = "shmem_frame_" + std::to_string(seq);
  msg.height = static_cast<std::uint32_t>(image_height_);
  msg.width = static_cast<std::uint32_t>(image_width_);
  msg.encoding = encoding_;
  msg.is_bigendian = 0;
  msg.step = static_cast<std::uint32_t>(image_width_);

  if (use_shm_) {
    // Shared-memory backend allocation
    msg.data = shmem_buffer::allocate_buffer(payload_size);
    auto write_handle = shmem_buffer::from_output_buffer(msg.data);
    fill_test_pattern(write_handle.get_ptr(), payload_size, seq);
  } else {
    // Non-shm path: Standard ROS 2 CPU buffer
    msg.data = rosidl::Buffer<std::uint8_t>(payload_size);
    fill_test_pattern(msg.data.data(), payload_size, seq);
  }

  // Keep recent published messages alive in pool ring buffer
  message_history_.push_back(std::move(msg));
  if (static_cast<int>(message_history_.size()) > history_depth_) {
    message_history_.pop_front();
  }

  publisher_->publish(message_history_.back());
  sequence_ = seq;

  std_msgs::msg::UInt32 count_msg;
  count_msg.data = static_cast<std::uint32_t>(sequence_);
  count_publisher_->publish(count_msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Published msg #%zu (%zux%zu, %zu B, backend: %s)",
    sequence_, static_cast<std::size_t>(image_width_), static_cast<std::size_t>(image_height_),
    payload_size, message_history_.back().data.get_backend_type().c_str());
}

}  // namespace shmem_buffer_backend_test

RCLCPP_COMPONENTS_REGISTER_NODE(shmem_buffer_backend_test::ShmemImagePublisherNode)
