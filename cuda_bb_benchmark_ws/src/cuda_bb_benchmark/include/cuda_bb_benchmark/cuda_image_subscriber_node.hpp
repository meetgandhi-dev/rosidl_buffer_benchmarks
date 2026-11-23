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

#ifndef CUDA_BB_BENCHMARK__CUDA_IMAGE_SUBSCRIBER_NODE_HPP_
#define CUDA_BB_BENCHMARK__CUDA_IMAGE_SUBSCRIBER_NODE_HPP_

#include <cuda_runtime.h>

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
#include "std_msgs/msg/u_int32.hpp"

namespace cuda_bb_benchmark
{

class CudaImageSubscriberNode : public rclcpp::Node
{
public:
  explicit CudaImageSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CudaImageSubscriberNode();

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

  cudaStream_t stream_{nullptr};
  int cuda_device_id_{0};

  std::string topic_name_{"test_cuda_image"};
  std::string result_prefix_{"cuda_test"};
  bool require_cuda_{true};
  bool use_cuda_{true};
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
  std::string last_backend_{"cuda"};
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
};

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__CUDA_IMAGE_SUBSCRIBER_NODE_HPP_
