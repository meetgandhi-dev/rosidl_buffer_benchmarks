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

#ifndef CUDA_BB_BENCHMARK__CUDA_IMAGE_PUBLISHER_NODE_HPP_
#define CUDA_BB_BENCHMARK__CUDA_IMAGE_PUBLISHER_NODE_HPP_

#include <cuda_runtime.h>

#include <chrono>
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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "cuda_bb_benchmark/system_monitor.hpp"

namespace cuda_bb_benchmark
{

class CudaImagePublisherNode : public rclcpp::Node
{
public:
  explicit CudaImagePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CudaImagePublisherNode();

  std::size_t get_published_count() const {return sequence_;}

private:
  void publish_timer_callback();
  void write_publisher_summary(double actual_fps, std::size_t total_sent);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_report_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  cudaStream_t stream_{nullptr};
  int cuda_device_id_{0};

  std::string topic_name_{"test_cuda_image"};
  double publish_rate_hz_{10.0};
  int image_width_{640};
  int image_height_{480};
  std::string encoding_{"mono8"};
  int history_depth_{20};
  int max_messages_{0};
  int warmup_messages_{10};
  bool wait_for_subscriber_{false};
  bool auto_exit_{false};
  bool use_cuda_{true};
  std::string summary_csv_path_{""};

  std::size_t sequence_{0};
  std::deque<sensor_msgs::msg::Image> message_history_;

  std::chrono::steady_clock::time_point benchmark_start_time_{};
  std::chrono::steady_clock::time_point benchmark_end_time_{};

  SystemMonitor proc_monitor_;
  bool summary_written_{false};
};

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__CUDA_IMAGE_PUBLISHER_NODE_HPP_
