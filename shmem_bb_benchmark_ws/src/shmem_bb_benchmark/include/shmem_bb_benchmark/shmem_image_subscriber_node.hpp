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

#ifndef SHMEM_BB_BENCHMARK__SHMEM_IMAGE_SUBSCRIBER_NODE_HPP_
#define SHMEM_BB_BENCHMARK__SHMEM_IMAGE_SUBSCRIBER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace shmem_bb_benchmark
{

class ShmemImageSubscriberNode : public rclcpp::Node
{
public:
  explicit ShmemImageSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ShmemImageSubscriberNode();

  std::size_t get_received_count() const {return received_count_;}
  bool is_all_valid() const {return all_valid_;}

private:
  void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void write_summary_report(const std::string & backend, uint32_t width, uint32_t height);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr backend_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr content_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr all_valid_pub_;

  std::string topic_name_{"test_shmem_image"};
  std::string result_prefix_{"shmem_test"};
  bool require_shmem_{true};
  bool use_shm_{true};
  bool verify_content_{true};
  bool publish_diagnostics_{true};

  int max_messages_{0};
  bool auto_exit_{false};
  double target_hz_{0.0};
  std::string latency_csv_path_{""};
  std::string summary_csv_path_{""};
  std::vector<double> latencies_ms_;

  std::size_t received_count_{0};
  bool all_valid_{true};
  bool summary_written_{false};
  std::string last_backend_{"shmem"};
  uint32_t last_width_{0};
  uint32_t last_height_{0};
};

}  // namespace shmem_bb_benchmark

#endif  // SHMEM_BB_BENCHMARK__SHMEM_IMAGE_SUBSCRIBER_NODE_HPP_
