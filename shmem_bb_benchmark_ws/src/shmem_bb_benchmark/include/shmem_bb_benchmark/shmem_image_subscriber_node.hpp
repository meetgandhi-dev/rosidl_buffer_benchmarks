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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "shmem_bb_benchmark/system_monitor.hpp"

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
  void pub_report_callback(std_msgs::msg::String::ConstSharedPtr msg);
  void write_summary_report(const std::string & backend, uint32_t width, uint32_t height);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pub_report_sub_;
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
  int warmup_messages_{10};
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

  // Frame and sequence tracking for drop detection
  std::size_t last_seq_{0};
  std::size_t first_bench_seq_{0};
  std::size_t last_bench_seq_{0};
  std::size_t dropped_frames_{0};

  // Steady-clock throughput tracking
  std::chrono::steady_clock::time_point sub_first_recv_time_{};
  std::chrono::steady_clock::time_point sub_last_recv_time_{};
  int64_t pub_first_stamp_ns_{0};
  int64_t pub_last_stamp_ns_{0};

  // Telemetry from Publisher Report
  double pub_fps_{0.0};
  std::size_t pub_total_frames_{0};
  double pub_cpu_mean_pct_{0.0};
  double pub_cpu_max_pct_{0.0};
  double pub_rss_mean_mb_{0.0};
  double pub_rss_peak_mb_{0.0};
  bool pub_report_received_{false};

  // System resource monitor
  SystemMonitor sys_monitor_;
};

}  // namespace shmem_bb_benchmark

#endif  // SHMEM_BB_BENCHMARK__SHMEM_IMAGE_SUBSCRIBER_NODE_HPP_
