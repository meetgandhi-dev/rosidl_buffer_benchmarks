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

#ifndef SHMEM_BUFFER_BACKEND_TEST__SHMEM_IMAGE_PUBLISHER_NODE_HPP_
#define SHMEM_BUFFER_BACKEND_TEST__SHMEM_IMAGE_PUBLISHER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace shmem_buffer_backend_test
{

class ShmemImagePublisherNode : public rclcpp::Node
{
public:
  explicit ShmemImagePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ShmemImagePublisherNode() = default;

  std::size_t get_published_count() const {return sequence_;}

private:
  void publish_timer_callback();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  std::string topic_name_{"test_shmem_image"};
  double publish_rate_hz_{10.0};
  int image_width_{640};
  int image_height_{480};
  std::string encoding_{"mono8"};
  int history_depth_{20};
  int max_messages_{0};
  bool wait_for_subscriber_{false};
  bool auto_exit_{false};
  bool use_shm_{true};

  std::size_t sequence_{0};
  std::deque<sensor_msgs::msg::Image> message_history_;
};

}  // namespace shmem_buffer_backend_test

#endif  // SHMEM_BUFFER_BACKEND_TEST__SHMEM_IMAGE_PUBLISHER_NODE_HPP_
