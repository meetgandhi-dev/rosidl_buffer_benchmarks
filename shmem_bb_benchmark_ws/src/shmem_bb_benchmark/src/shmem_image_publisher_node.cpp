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

#include "shmem_bb_benchmark/shmem_image_publisher_node.hpp"

#include <chrono>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "shmem_buffer/shmem_buffer_api.hpp"
#include "shmem_bb_benchmark/test_pattern_utils.hpp"

namespace shmem_bb_benchmark
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
  warmup_messages_ = declare_parameter<int>("warmup_messages", 10);
  wait_for_subscriber_ = declare_parameter<bool>("wait_for_subscriber", false);
  auto_exit_ = declare_parameter<bool>("auto_exit", false);
  use_shm_ = declare_parameter<bool>("use_shm", true);
  summary_csv_path_ = declare_parameter<std::string>("summary_csv_path", "");

  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, history_depth_);
  count_publisher_ = create_publisher<std_msgs::msg::UInt32>(topic_name_ + "/pub_count", 10);
  pub_report_publisher_ = create_publisher<std_msgs::msg::String>(topic_name_ + "/pub_report", 10);

  const auto timer_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&ShmemImagePublisherNode::publish_timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "ShmemImagePublisherNode initialized: topic='%s', rate=%.1f Hz, res=%dx%d, max_msgs=%d, warmup_msgs=%d, history_depth=%d",
    topic_name_.c_str(), publish_rate_hz_, image_width_, image_height_, max_messages_, warmup_messages_, history_depth_);
}

void ShmemImagePublisherNode::publish_timer_callback()
{
  if (!rclcpp::ok()) {
    if (timer_) {
      timer_->cancel();
    }
    return;
  }

  const int total_messages = (max_messages_ > 0) ? (max_messages_ + warmup_messages_) : 0;
  if (total_messages > 0 && static_cast<int>(sequence_) >= total_messages) {
    timer_->cancel();
    message_history_.clear();
    benchmark_end_time_ = std::chrono::steady_clock::now();

    const double elapsed_sec = std::chrono::duration<double>(
      benchmark_end_time_ - benchmark_start_time_).count();
    const double actual_fps = (elapsed_sec > 0.0 && max_messages_ > 1) ?
      (max_messages_ - 1) / elapsed_sec : publish_rate_hz_;

    RCLCPP_INFO(
      get_logger(),
      "Finished publishing %zu messages (%d warm-up + %d benchmark). Actual Pub Rate: %.2f FPS. Stopping.",
      sequence_, warmup_messages_, max_messages_, actual_fps);
    write_publisher_summary(actual_fps, max_messages_);
    if (auto_exit_ && !shutdown_timer_) {
      shutdown_timer_ = create_wall_timer(
        std::chrono::milliseconds(2000),
        [this]() {
          shutdown_timer_->cancel();
          RCLCPP_INFO(get_logger(), "Publisher auto-exit triggered.");
          if (rclcpp::ok()) {
            rclcpp::shutdown();
          }
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

  // Measure timestamp using std::chrono::steady_clock for consistent monotonic pub-to-sub latency
  const auto steady_now = std::chrono::steady_clock::now();
  const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();

  if (seq == static_cast<std::size_t>(warmup_messages_ + 1)) {
    benchmark_start_time_ = steady_now;
    proc_monitor_.start();
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp.sec = static_cast<int32_t>(steady_ns / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(steady_ns % 1000000000LL);
  // Store sequence and steady timestamp nanoseconds in frame_id for robust tracking
  msg.header.frame_id = "frame_" + std::to_string(seq) + "_" + std::to_string(steady_ns);
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

  const bool is_warmup = (warmup_messages_ > 0 && static_cast<int>(sequence_) <= warmup_messages_);
  if (is_warmup) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published msg #%zu [WARM-UP %zu/%d] (%zux%zu, %zu B, backend: %s)",
      sequence_, sequence_, warmup_messages_,
      static_cast<std::size_t>(image_width_), static_cast<std::size_t>(image_height_),
      payload_size, message_history_.back().data.get_backend_type().c_str());
  } else {
    const std::size_t bench_idx = (warmup_messages_ > 0) ? (sequence_ - warmup_messages_) : sequence_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published msg #%zu [BENCHMARK %zu/%d] (%zux%zu, %zu B, backend: %s)",
      sequence_, bench_idx, max_messages_,
      static_cast<std::size_t>(image_width_), static_cast<std::size_t>(image_height_),
      payload_size, message_history_.back().data.get_backend_type().c_str());
  }
}

void ShmemImagePublisherNode::write_publisher_summary(double actual_fps, std::size_t total_sent)
{
  if (summary_written_) {
    return;
  }
  summary_written_ = true;
  proc_monitor_.stop();
  const auto sys_summary = proc_monitor_.get_summary();

  RCLCPP_INFO(get_logger(), "==========================================================================");
  RCLCPP_INFO(get_logger(), " PUBLISHER SUMMARY REPORT: topic='%s', %dx%d",
    topic_name_.c_str(), image_width_, image_height_);
  RCLCPP_INFO(get_logger(), "--------------------------------------------------------------------------");
  RCLCPP_INFO(get_logger(), " Throughput:  Target: %.1f Hz | Actual Pub: %.2f FPS | Total Published: %zu",
    publish_rate_hz_, actual_fps, total_sent);
  RCLCPP_INFO(get_logger(), " Resources:   Publisher CPU Util: Mean: %.1f%% | Max: %.1f%%",
    sys_summary.process_cpu_mean_pct, sys_summary.process_cpu_max_pct);
  RCLCPP_INFO(get_logger(), "              Publisher RSS:      Mean: %.1f MB | Peak: %.1f MB",
    sys_summary.process_rss_mean_mb, sys_summary.process_rss_peak_mb);
  RCLCPP_INFO(get_logger(), "              System RAM Used:    %.1f MB",
    sys_summary.system_ram_mean_mb);
  RCLCPP_INFO(get_logger(), "==========================================================================");

  // Publish report to subscriber
  if (pub_report_publisher_) {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "actual_pub_fps=" << actual_fps
       << ";total_published=" << total_sent
       << ";pub_cpu_mean=" << sys_summary.process_cpu_mean_pct
       << ";pub_cpu_max=" << sys_summary.process_cpu_max_pct
       << ";pub_rss_mean=" << sys_summary.process_rss_mean_mb
       << ";pub_rss_peak=" << sys_summary.process_rss_peak_mb
       << ";system_ram=" << sys_summary.system_ram_mean_mb;
    msg.data = ss.str();
    pub_report_publisher_->publish(msg);
  }

  // Write CSV if path provided
  if (!summary_csv_path_.empty()) {
    std::ofstream csv(summary_csv_path_);
    if (csv.is_open()) {
      csv << "actual_pub_fps,total_published_frames,pub_cpu_mean_pct,pub_cpu_max_pct,pub_rss_mean_mb,pub_rss_peak_mb,system_ram_mb\n"
          << std::fixed << std::setprecision(2) << actual_fps << ","
          << total_sent << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_cpu_mean_pct << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_cpu_max_pct << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_rss_mean_mb << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_rss_peak_mb << ","
          << std::fixed << std::setprecision(1) << sys_summary.system_ram_mean_mb << "\n";
    }
  }
}

}  // namespace shmem_bb_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(shmem_bb_benchmark::ShmemImagePublisherNode)
