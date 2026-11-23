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

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
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
              if (rclcpp::ok()) {
                rclcpp::shutdown();
              }
              std::thread([]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                _exit(0);
              }).detach();
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
  sys_monitor_.stop();
  if (!summary_written_ && !latencies_ms_.empty() && !summary_csv_path_.empty()) {
    write_summary_report(last_backend_, last_width_, last_height_);
  }
}

void ShmemImageSubscriberNode::image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!rclcpp::ok()) {
    return;
  }

  const auto steady_now = std::chrono::steady_clock::now();
  const int64_t sub_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();

  ++received_count_;

  const std::string backend = msg->data.get_backend_type();
  last_backend_ = backend;
  last_width_ = msg->width;
  last_height_ = msg->height;

  // Extract publisher steady_clock timestamp from header.stamp
  int64_t pub_steady_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);

  // Extract sequence number from frame_id (format: "frame_<seq>_<steady_ns>" or "shmem_frame_<seq>")
  std::size_t seq = received_count_;
  const auto first_underscore = msg->header.frame_id.find('_');
  if (first_underscore != std::string::npos) {
    const auto second_underscore = msg->header.frame_id.find('_', first_underscore + 1);
    const auto seq_str = (second_underscore != std::string::npos) ?
      msg->header.frame_id.substr(first_underscore + 1, second_underscore - first_underscore - 1) :
      msg->header.frame_id.substr(first_underscore + 1);
    try {
      seq = std::stoull(seq_str);
    } catch (...) {
      seq = received_count_;
    }
  }

  // Frame drop tracking
  if (last_seq_ > 0 && seq > last_seq_ + 1) {
    const std::size_t dropped = seq - last_seq_ - 1;
    dropped_frames_ += dropped;
    RCLCPP_WARN(
      get_logger(),
      "Frame drop detected: expected msg #%zu, got #%zu (%zu frame(s) lost)",
      last_seq_ + 1, seq, dropped);
  }
  last_seq_ = seq;

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

  // Consistent monotonic latency calculation via steady_clock
  const double latency_ms = (sub_steady_ns >= pub_steady_ns) ?
    static_cast<double>(sub_steady_ns - pub_steady_ns) / 1000000.0 : 0.0;
  const bool is_warmup = (warmup_messages_ > 0 && static_cast<int>(seq) <= warmup_messages_);

  if (is_warmup) {
    RCLCPP_INFO(
      get_logger(),
      "Received warm-up msg #%zu/%d (%zux%zu, %zu B, backend: '%s', latency: %.3f ms, valid: %s) [ignored for benchmark]",
      seq, warmup_messages_, static_cast<std::size_t>(msg->width),
      static_cast<std::size_t>(msg->height), msg->data.size(), backend.c_str(),
      latency_ms, (backend_valid && content_valid) ? "YES" : "NO");
  } else {
    if (latencies_ms_.empty()) {
      first_bench_seq_ = seq;
      sub_first_recv_time_ = steady_now;
      pub_first_stamp_ns_ = pub_steady_ns;
      sys_monitor_.start();
    }
    last_bench_seq_ = seq;
    sub_last_recv_time_ = steady_now;
    pub_last_stamp_ns_ = pub_steady_ns;

    latencies_ms_.push_back(latency_ms);
    const std::size_t bench_idx = latencies_ms_.size();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received msg #%zu [BENCHMARK %zu/%d] (%zux%zu, %zu B, backend: '%s', latency: %.3f ms, valid: %s)",
      seq, bench_idx, max_messages_, static_cast<std::size_t>(msg->width),
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
    write_summary_report(backend, msg->width, msg->height);

    if (auto_exit_) {
      if (watchdog_timer_) {
        watchdog_timer_->cancel();
      }
      RCLCPP_INFO(get_logger(), "Auto-exit triggered. Shutting down subscriber.");
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      // Safety watchdog: ensure process terminates and does not hang on lingering background threads
      std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        _exit(0);
      }).detach();
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
  sys_monitor_.stop();
  const auto sys_summary = sys_monitor_.get_summary();

  auto samples = latencies_ms_;
  std::sort(samples.begin(), samples.end());

  auto percentile = [&samples](double p) -> double {
    if (samples.empty()) {return 0.0;}
    if (samples.size() == 1) {return samples[0];}
    double rank = p * static_cast<double>(samples.size() - 1);
    std::size_t lower = static_cast<std::size_t>(rank);
    std::size_t upper = std::min(lower + 1, samples.size() - 1);
    double weight = rank - static_cast<double>(lower);
    return samples[lower] * (1.0 - weight) + samples[upper] * weight;
  };

  const double min_ms = samples.empty() ? 0.0 : samples.front();
  const double max_ms = samples.empty() ? 0.0 : samples.back();
  const double p50_ms = percentile(0.50);
  const double p95_ms = percentile(0.95);
  const double p99_ms = percentile(0.99);

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

  // Outlier detection: frames exceeding 3-sigma
  const double outlier_threshold_ms = mean_ms + 3.0 * std_dev_ms;
  std::size_t outlier_count = 0;
  for (double v : samples) {
    if (v > outlier_threshold_ms) {
      ++outlier_count;
    }
  }

  const std::size_t measured_frames = latencies_ms_.size();
  const std::size_t size_bytes = static_cast<std::size_t>(width) * height;

  // Frame loss calculation
  const std::size_t total_sent_in_window = (last_bench_seq_ >= first_bench_seq_) ?
    (last_bench_seq_ - first_bench_seq_ + 1) : measured_frames;
  const std::size_t lost_frames = (total_sent_in_window > measured_frames) ?
    (total_sent_in_window - measured_frames) : 0;
  const double frame_loss_pct = (total_sent_in_window > 0) ?
    (100.0 * static_cast<double>(lost_frames) / static_cast<double>(total_sent_in_window)) : 0.0;

  // Actual Publishing FPS (based on steady_clock publisher timestamp delta)
  const double pub_duration_s = (pub_last_stamp_ns_ > pub_first_stamp_ns_) ?
    static_cast<double>(pub_last_stamp_ns_ - pub_first_stamp_ns_) / 1e9 : 0.0;
  const double actual_pub_hz = (pub_duration_s > 0.0 && total_sent_in_window > 1) ?
    static_cast<double>(total_sent_in_window - 1) / pub_duration_s : target_hz_;

  // Actual Receiving FPS (based on steady_clock subscriber receipt delta)
  const double sub_duration_s = std::chrono::duration<double>(
    sub_last_recv_time_ - sub_first_recv_time_).count();
  const double actual_recv_hz = (sub_duration_s > 0.0 && measured_frames > 1) ?
    static_cast<double>(measured_frames - 1) / sub_duration_s : 0.0;

  RCLCPP_INFO(get_logger(), "==========================================================================");
  RCLCPP_INFO(get_logger(), " BENCHMARK SUMMARY REPORT: backend='%s', %ux%u (%zu B)",
    backend.c_str(), width, height, size_bytes);
  RCLCPP_INFO(get_logger(), "--------------------------------------------------------------------------");
  RCLCPP_INFO(get_logger(), " Throughput:  Target: %.1f Hz | Actual Pub: %.2f Hz | Actual Recv: %.2f Hz",
    target_hz_, actual_pub_hz, actual_recv_hz);
  RCLCPP_INFO(get_logger(), " Frames:      Measured: %zu | Total Sent: %zu | Lost: %zu (%.2f%% loss) | Warm-up: %d",
    measured_frames, total_sent_in_window, lost_frames, frame_loss_pct, warmup_messages_);
  RCLCPP_INFO(get_logger(), " Latency (ms):");
  RCLCPP_INFO(get_logger(), "   P50:  %.3f | P95: %.3f | P99: %.3f", p50_ms, p95_ms, p99_ms);
  RCLCPP_INFO(get_logger(), "   Mean: %.3f | Min: %.3f | Max: %.3f | StdDev: %.3f",
    mean_ms, min_ms, max_ms, std_dev_ms);
  RCLCPP_INFO(get_logger(), " Outliers:    %zu frame(s) exceeded 3-sigma (%.3f ms) [Max outlier: %.3f ms]",
    outlier_count, outlier_threshold_ms, max_ms);
  RCLCPP_INFO(get_logger(), " System Resources:");
  RCLCPP_INFO(get_logger(), "   CPU Util:    Mean: %.1f%% | Max: %.1f%%",
    sys_summary.cpu_util_mean_pct, sys_summary.cpu_util_max_pct);
  RCLCPP_INFO(get_logger(), "   RAM Usage:   Process RSS: %.1f MB (Peak: %.1f MB) | System Used: %.1f MB",
    sys_summary.process_rss_mean_mb, sys_summary.process_rss_peak_mb, sys_summary.system_ram_mean_mb);
  RCLCPP_INFO(get_logger(), "   CPU Clock:   Mean: %.0f MHz | Max: %.0f MHz",
    sys_summary.cpu_clock_mean_mhz, sys_summary.cpu_clock_max_mhz);
  if (sys_summary.power_available) {
    RCLCPP_INFO(get_logger(), "   Power Draw:  Mean: %.2f W | Max: %.2f W",
      sys_summary.power_mean_watts, sys_summary.power_max_watts);
  } else {
    RCLCPP_INFO(get_logger(), "   Power Draw:  N/A (no readable power sensor)");
  }
  if (sys_summary.gpu_available) {
    RCLCPP_INFO(get_logger(), "   GPU Util:    Mean: %.1f%% | Clock: %.0f MHz | Power: %.2f W",
      sys_summary.gpu_util_mean_pct, sys_summary.gpu_clock_mean_mhz, sys_summary.gpu_power_mean_watts);
  }
  RCLCPP_INFO(get_logger(), "==========================================================================");

  if (!summary_csv_path_.empty()) {
    std::ifstream check_file(summary_csv_path_);
    const bool exists = check_file.good();
    check_file.close();

    std::ofstream csv(summary_csv_path_, std::ios::app);
    if (csv.is_open()) {
      if (!exists) {
        csv << "backend,size_bytes,resolution,target_hz,actual_pub_hz,actual_recv_hz,"
            << "total_sent,received_frames,lost_frames,loss_pct,mean_latency_ms,"
            << "min_latency_ms,p50_latency_ms,p95_latency_ms,p99_latency_ms,max_latency_ms,"
            << "std_dev_ms,outliers_3sigma_count,"
            << "cpu_util_mean_pct,cpu_util_max_pct,process_rss_mean_mb,process_rss_peak_mb,"
            << "system_ram_mb,cpu_clock_mean_mhz,cpu_clock_max_mhz,power_mean_w,power_max_w,"
            << "gpu_util_pct,gpu_clock_mhz,gpu_power_w\n";
      }
      csv << backend << ","
          << size_bytes << ","
          << width << "x" << height << ","
          << std::fixed << std::setprecision(1) << target_hz_ << ","
          << std::fixed << std::setprecision(2) << actual_pub_hz << ","
          << std::fixed << std::setprecision(2) << actual_recv_hz << ","
          << total_sent_in_window << ","
          << measured_frames << ","
          << lost_frames << ","
          << std::fixed << std::setprecision(2) << frame_loss_pct << ","
          << std::fixed << std::setprecision(3) << mean_ms << ","
          << std::fixed << std::setprecision(3) << min_ms << ","
          << std::fixed << std::setprecision(3) << p50_ms << ","
          << std::fixed << std::setprecision(3) << p95_ms << ","
          << std::fixed << std::setprecision(3) << p99_ms << ","
          << std::fixed << std::setprecision(3) << max_ms << ","
          << std::fixed << std::setprecision(3) << std_dev_ms << ","
          << outlier_count << ","
          << std::fixed << std::setprecision(1) << sys_summary.cpu_util_mean_pct << ","
          << std::fixed << std::setprecision(1) << sys_summary.cpu_util_max_pct << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_rss_mean_mb << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_rss_peak_mb << ","
          << std::fixed << std::setprecision(1) << sys_summary.system_ram_mean_mb << ","
          << std::fixed << std::setprecision(0) << sys_summary.cpu_clock_mean_mhz << ","
          << std::fixed << std::setprecision(0) << sys_summary.cpu_clock_max_mhz << ","
          << std::fixed << std::setprecision(2) << sys_summary.power_mean_watts << ","
          << std::fixed << std::setprecision(2) << sys_summary.power_max_watts << ","
          << std::fixed << std::setprecision(1) << (sys_summary.gpu_available ? sys_summary.gpu_util_mean_pct : 0.0) << ","
          << std::fixed << std::setprecision(0) << (sys_summary.gpu_available ? sys_summary.gpu_clock_mean_mhz : 0.0) << ","
          << std::fixed << std::setprecision(2) << (sys_summary.gpu_available ? sys_summary.gpu_power_mean_watts : 0.0) << "\n";
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
