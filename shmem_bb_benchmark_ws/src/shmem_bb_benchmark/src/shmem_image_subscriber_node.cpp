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

#include "shmem_bb_benchmark/shmem_image_subscriber_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/subscription_options.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "shmem_buffer/shmem_buffer_api.hpp"
#include "shmem_bb_benchmark/test_pattern_utils.hpp"

namespace shmem_bb_benchmark
{

ShmemImageSubscriberNode::ShmemImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("shmem_image_subscriber", options)
{
  topic_name_ = declare_parameter<std::string>("topic", "test_shmem_image");
  result_prefix_ = declare_parameter<std::string>("result_prefix", "shmem_test");
  require_shmem_ = declare_parameter<bool>("require_shmem", true);
  use_shm_ = declare_parameter<bool>("use_shm", true);
  verify_content_ = declare_parameter<bool>("verify_content", true);
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  max_messages_ = declare_parameter<int>("max_messages", 0);
  warmup_messages_ = declare_parameter<int>("warmup_messages", 10);
  auto_exit_ = declare_parameter<bool>("auto_exit", false);
  target_hz_ = declare_parameter<double>("target_hz", 0.0);
  latency_csv_path_ = declare_parameter<std::string>("latency_csv_path", "");
  summary_csv_path_ = declare_parameter<std::string>("summary_csv_path", "");

  int history_depth = declare_parameter<int>("history_depth", 20);

  rclcpp::SubscriptionOptions sub_options;
  if (use_shm_ && require_shmem_) {
    // Enforce shared-memory backend; reject fallback to CPU copies
    sub_options.acceptable_buffer_backends = "shmem";
  }

  subscription_ = create_subscription<sensor_msgs::msg::Image>(
    topic_name_, history_depth,
    std::bind(&ShmemImageSubscriberNode::image_callback, this, std::placeholders::_1),
    sub_options);

  if (publish_diagnostics_) {
    count_pub_ = create_publisher<std_msgs::msg::UInt32>(result_prefix_ + "/count", 10);
    backend_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      result_prefix_ + "/backend_valid", 10);
    content_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      result_prefix_ + "/content_valid", 10);
    all_valid_pub_ = create_publisher<std_msgs::msg::Bool>(result_prefix_ + "/all_valid", 10);
  }

  if (auto_exit_ || max_messages_ > 0) {
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        if (!latencies_ms_.empty() && max_messages_ > 0) {
          if (subscription_->get_publisher_count() == 0u) {
            RCLCPP_INFO(
              get_logger(),
              "Publisher disconnected. Finalizing subscriber with %zu measured frames (%zu total received).",
              latencies_ms_.size(), received_count_);
            write_summary_report(last_backend_, last_width_, last_height_);
            if (auto_exit_) {
              watchdog_timer_->cancel();
              rclcpp::shutdown();
            }
          }
        }
      });
  }

  RCLCPP_INFO(
    get_logger(),
    "ShmemImageSubscriberNode initialized: topic='%s', max_msgs=%d, warmup_msgs=%d, require_shmem=%s, verify_content=%s",
    topic_name_.c_str(), max_messages_, warmup_messages_,
    require_shmem_ ? "true" : "false",
    verify_content_ ? "true" : "false");
}

ShmemImageSubscriberNode::~ShmemImageSubscriberNode()
{
  if (!summary_written_ && !latencies_ms_.empty() && !summary_csv_path_.empty()) {
    write_summary_report(last_backend_, last_width_, last_height_);
  }
}

void ShmemImageSubscriberNode::image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ++received_count_;

  const std::string backend = msg->data.get_backend_type();
  last_backend_ = backend;
  last_width_ = msg->width;
  last_height_ = msg->height;

  // Extract sequence number from frame_id (e.g. "shmem_frame_123")
  std::size_t seq = received_count_;
  const auto last_underscore = msg->header.frame_id.rfind('_');
  if (last_underscore != std::string::npos &&
    last_underscore + 1 < msg->header.frame_id.size())
  {
    try {
      seq = std::stoull(msg->header.frame_id.substr(last_underscore + 1));
    } catch (...) {
      seq = received_count_;
    }
  }

  const bool backend_valid =
    use_shm_ ? (!require_shmem_ || backend == "shmem") : (backend == "cpu");
  if (!backend_valid) {
    all_valid_ = false;
    RCLCPP_WARN(
      get_logger(),
      "Message #%zu backend mismatch: expected '%s', got '%s'",
      received_count_, use_shm_ ? "shmem" : "cpu", backend.c_str());
  }

  bool content_valid = true;
  if (verify_content_) {
    try {
      const std::size_t payload_size = msg->data.size();
      const std::uint8_t * payload_ptr = nullptr;
      std::optional<shmem_buffer::ReadHandle> read_handle;

      if (backend == "shmem") {
        read_handle = shmem_buffer::from_input_buffer(msg->data);
        payload_ptr = read_handle->get_ptr();
      } else {
        payload_ptr = msg->data.data();
      }

      content_valid = verify_test_pattern(payload_ptr, payload_size, seq);
    } catch (const std::exception & e) {
      content_valid = false;
      RCLCPP_ERROR(
        get_logger(),
        "Exception reading buffer on msg #%zu: %s",
        received_count_, e.what());
    }
  }

  if (!content_valid) {
    all_valid_ = false;
    RCLCPP_ERROR(
      get_logger(), "Message #%zu content verification failed!", received_count_);
  }

  const double latency_ms = (now() - msg->header.stamp).seconds() * 1000.0;
  const bool is_warmup = (warmup_messages_ > 0 && static_cast<int>(received_count_) <= warmup_messages_);

  if (is_warmup) {
    RCLCPP_INFO(
      get_logger(),
      "Received warm-up msg #%zu/%d (%zux%zu, %zu B, backend: '%s', latency: %.2f ms, valid: %s) [ignored for benchmark]",
      received_count_, warmup_messages_, static_cast<std::size_t>(msg->width),
      static_cast<std::size_t>(msg->height), msg->data.size(), backend.c_str(),
      latency_ms, (backend_valid && content_valid) ? "YES" : "NO");
  } else {
    latencies_ms_.push_back(latency_ms);
    const std::size_t bench_idx = latencies_ms_.size();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received msg #%zu [BENCHMARK %zu/%d] (%zux%zu, %zu B, backend: '%s', latency: %.2f ms, valid: %s)",
      received_count_, bench_idx, max_messages_, static_cast<std::size_t>(msg->width),
      static_cast<std::size_t>(msg->height), msg->data.size(), backend.c_str(),
      latency_ms, (backend_valid && content_valid) ? "YES" : "NO");
  }

  if (publish_diagnostics_) {
    std_msgs::msg::UInt32 count_msg;
    count_msg.data = static_cast<std::uint32_t>(received_count_);
    count_pub_->publish(count_msg);

    std_msgs::msg::Bool backend_msg;
    backend_msg.data = backend_valid;
    backend_valid_pub_->publish(backend_msg);

    std_msgs::msg::Bool content_msg;
    content_msg.data = content_valid;
    content_valid_pub_->publish(content_msg);

    std_msgs::msg::Bool all_msg;
    all_msg.data = all_valid_ && backend_valid && content_valid;
    all_valid_pub_->publish(all_msg);
  }

  const int total_expected = (max_messages_ > 0) ? (max_messages_ + warmup_messages_) : 0;
  const bool is_complete =
    (max_messages_ > 0 &&
    (static_cast<int>(latencies_ms_.size()) >= max_messages_ ||
     static_cast<int>(received_count_) >= total_expected ||
     seq >= static_cast<std::size_t>(total_expected)));

  if (is_complete) {
    if (seq > received_count_) {
      RCLCPP_WARN(
        get_logger(),
        "Sequence completed up to msg #%zu (%zu frames received, %zu dropped/skipped).",
        seq, received_count_, seq - received_count_);
    }
    write_summary_report(backend, msg->width, msg->height);

    if (auto_exit_) {
      if (watchdog_timer_) {
        watchdog_timer_->cancel();
      }
      RCLCPP_INFO(get_logger(), "Auto-exit triggered. Shutting down subscriber.");
      rclcpp::shutdown();
    }
  }
}

void ShmemImageSubscriberNode::write_summary_report(
  const std::string & backend, uint32_t width, uint32_t height)
{
  if (summary_written_ || latencies_ms_.empty()) {
    return;
  }
  summary_written_ = true;

  auto samples = latencies_ms_;
  std::sort(samples.begin(), samples.end());
  const double min_ms = samples.empty() ? 0.0 : samples.front();
  const double max_ms = samples.empty() ? 0.0 : samples.back();
  double sum = 0.0;
  for (double v : samples) {
    sum += v;
  }
  const double mean_ms = samples.empty() ? 0.0 : (sum / samples.size());
  double sq_diff = 0.0;
  for (double v : samples) {
    sq_diff += (v - mean_ms) * (v - mean_ms);
  }
  const double std_dev_ms = samples.empty() ? 0.0 : std::sqrt(sq_diff / samples.size());

  const std::size_t measured_frames = latencies_ms_.size();
  const std::size_t size_bytes = static_cast<std::size_t>(width) * height;

  RCLCPP_INFO(
    get_logger(),
    "=== Benchmark: %zu measured frames (excluded %d warm-up frames), %ux%u (%zu B), Target: %.1f Hz ===",
    measured_frames, warmup_messages_, width, height, size_bytes, target_hz_);
  RCLCPP_INFO(
    get_logger(),
    "Latency (ms) -> Mean: %.3f | Min: %.3f | Max: %.3f | StdDev: %.3f",
    mean_ms, min_ms, max_ms, std_dev_ms);

  if (!summary_csv_path_.empty()) {
    std::ifstream check_file(summary_csv_path_);
    const bool exists = check_file.good();
    check_file.close();

    std::ofstream csv(summary_csv_path_, std::ios::app);
    if (csv.is_open()) {
      if (!exists) {
        csv << "backend,size_bytes,resolution,target_hz,frame_cap,received_frames,avg_latency_ms,"
            << "min_latency_ms,max_latency_ms,std_dev_ms\n";
      }
      csv << backend << ","
          << size_bytes << ","
          << width << "x" << height << ","
          << target_hz_ << ","
          << max_messages_ << ","
          << measured_frames << ","
          << std::fixed << std::setprecision(3)
          << mean_ms << ","
          << min_ms << ","
          << max_ms << ","
          << std_dev_ms << "\n";
    }
  }

  if (!latency_csv_path_.empty()) {
    std::ofstream csv(latency_csv_path_);
    if (csv.is_open()) {
      csv << "frame,size_bytes,latency_ms\n";
      for (std::size_t i = 0; i < latencies_ms_.size(); ++i) {
        csv << (i + 1) << "," << size_bytes << ","
            << std::fixed << std::setprecision(3) << latencies_ms_[i] << "\n";
      }
    }
  }
}

}  // namespace shmem_bb_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(shmem_bb_benchmark::ShmemImageSubscriberNode)
