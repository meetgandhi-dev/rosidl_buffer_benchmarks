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

#include "cuda_bb_benchmark/cuda_image_subscriber_node.hpp"

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
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/subscription_options.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_bb_benchmark/test_pattern_utils.hpp"

namespace cuda_bb_benchmark
{

CudaImageSubscriberNode::CudaImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("cuda_image_subscriber", options)
{
  topic_name_ = declare_parameter<std::string>("topic", "test_cuda_image");
  result_prefix_ = declare_parameter<std::string>("result_prefix", "cuda_test");
  require_cuda_ = declare_parameter<bool>("require_cuda", true);
  use_cuda_ = declare_parameter<bool>("use_cuda", true);
  verify_content_ = declare_parameter<bool>("verify_content", true);
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  max_messages_ = declare_parameter<int>("max_messages", 0);
  warmup_messages_ = declare_parameter<int>("warmup_messages", 10);
  auto_exit_ = declare_parameter<bool>("auto_exit", false);
  target_hz_ = declare_parameter<double>("target_hz", 0.0);
  latency_csv_path_ = declare_parameter<std::string>("latency_csv_path", "");
  summary_csv_path_ = declare_parameter<std::string>("summary_csv_path", "");
  cuda_device_id_ = declare_parameter<int>("cuda_device_id", 0);

  int history_depth = declare_parameter<int>("history_depth", 20);

  if (use_cuda_) {
    cudaSetDevice(cuda_device_id_);
    cudaStreamCreate(&stream_);
  }

  rclcpp::SubscriptionOptions sub_options;
  if (use_cuda_ && require_cuda_) {
    // Require CUDA zero-copy backend; reject fallback to CPU copies
    sub_options.acceptable_buffer_backends = "cuda";
  }

  subscription_ = create_subscription<sensor_msgs::msg::Image>(
    topic_name_, history_depth,
    std::bind(&CudaImageSubscriberNode::image_callback, this, std::placeholders::_1),
    sub_options);

  pub_report_sub_ = create_subscription<std_msgs::msg::String>(
    topic_name_ + "/pub_report", 10,
    std::bind(&CudaImageSubscriberNode::pub_report_callback, this, std::placeholders::_1));

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
    "CudaImageSubscriberNode initialized: topic='%s', max_msgs=%d, warmup_msgs=%d, require_cuda=%s, verify_content=%s",
    topic_name_.c_str(), max_messages_, warmup_messages_,
    require_cuda_ ? "true" : "false",
    verify_content_ ? "true" : "false");
}

CudaImageSubscriberNode::~CudaImageSubscriberNode()
{
  sys_monitor_.stop();
  if (!summary_written_ && !latencies_ms_.empty() && !summary_csv_path_.empty()) {
    write_summary_report(last_backend_, last_width_, last_height_);
  }
  if (stream_ != nullptr) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

void CudaImageSubscriberNode::pub_report_callback(std_msgs::msg::String::ConstSharedPtr msg)
{
  std::istringstream ss(msg->data);
  std::string token;
  while (std::getline(ss, token, ';')) {
    auto eq = token.find('=');
    if (eq != std::string::npos) {
      auto key = token.substr(0, eq);
      auto val = token.substr(eq + 1);
      try {
        if (key == "actual_pub_fps") pub_fps_ = std::stod(val);
        else if (key == "total_published") pub_total_frames_ = std::stoull(val);
        else if (key == "pub_cpu_mean") pub_cpu_mean_pct_ = std::stod(val);
        else if (key == "pub_cpu_max") pub_cpu_max_pct_ = std::stod(val);
        else if (key == "pub_rss_mean") pub_rss_mean_mb_ = std::stod(val);
        else if (key == "pub_rss_peak") pub_rss_peak_mb_ = std::stod(val);
      } catch (...) {}
    }
  }
  pub_report_received_ = true;
}

void CudaImageSubscriberNode::image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!rclcpp::ok()) {
    return;
  }

  const auto cb_entry_time = std::chrono::steady_clock::now();
  const int64_t cb_entry_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    cb_entry_time.time_since_epoch()).count();

  ++received_count_;

  const std::string backend = msg->data.get_backend_type();
  last_backend_ = backend;
  last_width_ = msg->width;
  last_height_ = msg->height;

  // Extract publisher steady_clock timestamp from header.stamp
  int64_t pub_steady_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);

  // Extract sequence number from frame_id (format: "frame_<seq>_<steady_ns>" or "cuda_frame_<seq>")
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
    use_cuda_ ? (!require_cuda_ || backend == "cuda") : (backend == "cpu");
  if (!backend_valid) {
    all_valid_ = false;
    RCLCPP_WARN(
      get_logger(),
      "Message #%zu backend mismatch: expected '%s', got '%s'",
      received_count_, use_cuda_ ? "cuda" : "cpu", backend.c_str());
  }

  // Buffer readiness & synchronization
  int64_t buf_ready_steady_ns = cb_entry_steady_ns;
  std::optional<cuda_buffer_backend::ReadHandle> read_handle;
  if (backend == "cuda") {
    try {
      read_handle = cuda_buffer_backend::from_input_buffer(msg->data, stream_);
      // Required CUDA synchronization to ensure buffer is ready on device
      cudaStreamSynchronize(stream_);
      const auto buf_ready_time = std::chrono::steady_clock::now();
      buf_ready_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        buf_ready_time.time_since_epoch()).count();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Exception importing CUDA buffer on msg #%zu: %s",
        received_count_, e.what());
    }
  } else {
    // For CPU fallback, data buffer is already in host RAM
    const auto buf_ready_time = std::chrono::steady_clock::now();
    buf_ready_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      buf_ready_time.time_since_epoch()).count();
  }

  bool content_valid = true;
  if (verify_content_) {
    try {
      const std::size_t payload_size = msg->data.size();
      const uint8_t expected_val = generate_sequence_byte(seq);

      if (backend == "cuda" && read_handle.has_value()) {
        std::vector<uint8_t> host_buffer(payload_size);
        cudaMemcpyAsync(
          host_buffer.data(), read_handle->get_ptr(), payload_size,
          cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        for (std::size_t i = 0; i < payload_size; ++i) {
          if (host_buffer[i] != expected_val) {
            content_valid = false;
            RCLCPP_ERROR(
              get_logger(),
              "Content corruption at byte %zu: expected 0x%02x, got 0x%02x",
              i, expected_val, host_buffer[i]);
            break;
          }
        }
      } else {
        const uint8_t * payload_ptr = msg->data.data();
        for (std::size_t i = 0; i < payload_size; ++i) {
          if (payload_ptr[i] != expected_val) {
            content_valid = false;
            RCLCPP_ERROR(
              get_logger(),
              "Content corruption at byte %zu: expected 0x%02x, got 0x%02x",
              i, expected_val, payload_ptr[i]);
            break;
          }
        }
      }
    } catch (const std::exception & e) {
      content_valid = false;
      RCLCPP_ERROR(
        get_logger(),
        "Exception verifying buffer on msg #%zu: %s",
        received_count_, e.what());
    }
  }

  if (!content_valid) {
    all_valid_ = false;
    RCLCPP_ERROR(
      get_logger(), "Message #%zu content verification failed!", received_count_);
  }

  // Consistent monotonic latency calculation via steady_clock
  const double cb_latency_ms = (cb_entry_steady_ns >= pub_steady_ns) ?
    static_cast<double>(cb_entry_steady_ns - pub_steady_ns) / 1000000.0 : 0.0;
  const double buf_ready_latency_ms = (buf_ready_steady_ns >= pub_steady_ns) ?
    static_cast<double>(buf_ready_steady_ns - pub_steady_ns) / 1000000.0 : 0.0;

  const bool is_warmup = (warmup_messages_ > 0 && static_cast<int>(seq) <= warmup_messages_);

  if (is_warmup) {
    RCLCPP_INFO(
      get_logger(),
      "Received warm-up msg #%zu/%d (%zux%zu, %zu B, backend: '%s', cb_lat: %.3f ms, ready_lat: %.3f ms, valid: %s) [ignored for benchmark]",
      seq, warmup_messages_, static_cast<std::size_t>(msg->width),
      static_cast<std::size_t>(msg->height), msg->data.size(), backend.c_str(),
      cb_latency_ms, buf_ready_latency_ms, (backend_valid && content_valid) ? "YES" : "NO");
  } else {
    if (callback_latencies_ms_.empty()) {
      first_bench_seq_ = seq;
      sub_first_recv_time_ = cb_entry_time;
      pub_first_stamp_ns_ = pub_steady_ns;
      sys_monitor_.start();
    }
    last_bench_seq_ = seq;
    sub_last_recv_time_ = cb_entry_time;
    pub_last_stamp_ns_ = pub_steady_ns;

    callback_latencies_ms_.push_back(cb_latency_ms);
    buffer_ready_latencies_ms_.push_back(buf_ready_latency_ms);
    latencies_ms_.push_back(buf_ready_latency_ms);
    const std::size_t bench_idx = callback_latencies_ms_.size();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received msg #%zu [BENCHMARK %zu/%d] (%zux%zu, %zu B, backend: '%s', cb_lat: %.3f ms, ready_lat: %.3f ms, valid: %s)",
      seq, bench_idx, max_messages_, static_cast<std::size_t>(msg->width),
      static_cast<std::size_t>(msg->height), msg->data.size(), backend.c_str(),
      cb_latency_ms, buf_ready_latency_ms, (backend_valid && content_valid) ? "YES" : "NO");
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
      // Safety watchdog: ensure process terminates cleanly and does not hang on background threads
      std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        _exit(0);
      }).detach();
    }
  }
}

void CudaImageSubscriberNode::write_summary_report(
  const std::string & backend, uint32_t width, uint32_t height)
{
  if (summary_written_ || callback_latencies_ms_.empty()) {
    return;
  }
  summary_written_ = true;
  sys_monitor_.stop();
  const auto sys_summary = sys_monitor_.get_summary();

  struct LatencyStats
  {
    double min_ms{0.0};
    double max_ms{0.0};
    double p50_ms{0.0};
    double p95_ms{0.0};
    double p99_ms{0.0};
    double mean_ms{0.0};
    double std_dev_ms{0.0};
    std::size_t outliers_count{0};
  };

  auto calc_stats = [](const std::vector<double> & in_samples) -> LatencyStats {
    LatencyStats st;
    if (in_samples.empty()) {
      return st;
    }
    auto s = in_samples;
    std::sort(s.begin(), s.end());

    auto pct = [&s](double p) -> double {
      if (s.empty()) {return 0.0;}
      if (s.size() == 1) {return s[0];}
      double rank = p * static_cast<double>(s.size() - 1);
      std::size_t lower = static_cast<std::size_t>(rank);
      std::size_t upper = std::min(lower + 1, s.size() - 1);
      double weight = rank - static_cast<double>(lower);
      return s[lower] * (1.0 - weight) + s[upper] * weight;
    };

    st.min_ms = s.front();
    st.max_ms = s.back();
    st.p50_ms = pct(0.50);
    st.p95_ms = pct(0.95);
    st.p99_ms = pct(0.99);

    double sum = 0.0;
    for (double v : s) {
      sum += v;
    }
    st.mean_ms = sum / s.size();

    double sq_diff = 0.0;
    for (double v : s) {
      sq_diff += (v - st.mean_ms) * (v - st.mean_ms);
    }
    st.std_dev_ms = std::sqrt(sq_diff / s.size());

    const double thresh = st.mean_ms + 3.0 * st.std_dev_ms;
    for (double v : s) {
      if (v > thresh) {
        ++st.outliers_count;
      }
    }
    return st;
  };

  const auto cb_stats = calc_stats(callback_latencies_ms_);
  const auto br_stats = calc_stats(buffer_ready_latencies_ms_);
  const double sync_overhead_mean_ms = (br_stats.mean_ms >= cb_stats.mean_ms) ?
    (br_stats.mean_ms - cb_stats.mean_ms) : 0.0;

  const std::size_t measured_frames = callback_latencies_ms_.size();
  const std::size_t size_bytes = static_cast<std::size_t>(width) * height;

  // Frame loss calculation
  const std::size_t total_sent_in_window = (last_bench_seq_ >= first_bench_seq_) ?
    (last_bench_seq_ - first_bench_seq_ + 1) : measured_frames;
  const std::size_t total_published = (pub_total_frames_ > 0) ? pub_total_frames_ : total_sent_in_window;
  const std::size_t lost_frames = (total_published > measured_frames) ?
    (total_published - measured_frames) : 0;
  const double frame_loss_pct = (total_published > 0) ?
    (100.0 * static_cast<double>(lost_frames) / static_cast<double>(total_published)) : 0.0;

  // Actual Publishing FPS (prefer publisher report if available, else monotonic delta)
  const double pub_duration_s = (pub_last_stamp_ns_ > pub_first_stamp_ns_) ?
    static_cast<double>(pub_last_stamp_ns_ - pub_first_stamp_ns_) / 1e9 : 0.0;
  const double actual_pub_hz = (pub_fps_ > 0.0) ? pub_fps_ :
    ((pub_duration_s > 0.0 && total_sent_in_window > 1) ?
     static_cast<double>(total_sent_in_window - 1) / pub_duration_s : target_hz_);

  // Actual Receiving FPS (based on steady_clock subscriber receipt delta)
  const double sub_duration_s = std::chrono::duration<double>(
    sub_last_recv_time_ - sub_first_recv_time_).count();
  const double actual_recv_hz = (sub_duration_s > 0.0 && measured_frames > 1) ?
    static_cast<double>(measured_frames - 1) / sub_duration_s : 0.0;

  RCLCPP_INFO(get_logger(), "==========================================================================");
  RCLCPP_INFO(get_logger(), " BENCHMARK SUMMARY REPORT: backend='%s', %ux%u (%zu B)",
    backend.c_str(), width, height, size_bytes);
  RCLCPP_INFO(get_logger(), "--------------------------------------------------------------------------");
  RCLCPP_INFO(get_logger(), " Throughput:  Target: %.1f Hz | Actual Pub: %.2f FPS | Actual Recv: %.2f FPS",
    target_hz_, actual_pub_hz, actual_recv_hz);
  RCLCPP_INFO(get_logger(), " Frames:      Published: %zu | Received: %zu | Dropped: %zu (%.2f%% loss) | Warm-up: %d",
    total_published, measured_frames, lost_frames, frame_loss_pct, warmup_messages_);
  RCLCPP_INFO(get_logger(), " Latency (ms):");
  RCLCPP_INFO(get_logger(), "   [Callback Entry Latency]:");
  RCLCPP_INFO(get_logger(), "     P50:  %.3f | P95: %.3f | P99: %.3f", cb_stats.p50_ms, cb_stats.p95_ms, cb_stats.p99_ms);
  RCLCPP_INFO(get_logger(), "     Mean: %.3f | Min: %.3f | Max: %.3f | StdDev: %.3f",
    cb_stats.mean_ms, cb_stats.min_ms, cb_stats.max_ms, cb_stats.std_dev_ms);
  RCLCPP_INFO(get_logger(), "     Outliers (> 3-sigma): %zu frame(s)", cb_stats.outliers_count);
  RCLCPP_INFO(get_logger(), "   [Buffer-Ready Latency (after CUDA sync)]:");
  RCLCPP_INFO(get_logger(), "     P50:  %.3f | P95: %.3f | P99: %.3f", br_stats.p50_ms, br_stats.p95_ms, br_stats.p99_ms);
  RCLCPP_INFO(get_logger(), "     Mean: %.3f | Min: %.3f | Max: %.3f | StdDev: %.3f",
    br_stats.mean_ms, br_stats.min_ms, br_stats.max_ms, br_stats.std_dev_ms);
  RCLCPP_INFO(get_logger(), "     Outliers (> 3-sigma): %zu frame(s)", br_stats.outliers_count);
  RCLCPP_INFO(get_logger(), "   [CUDA Sync Overhead]: Mean: %.3f ms", sync_overhead_mean_ms);
  RCLCPP_INFO(get_logger(), " System Resources:");
  if (pub_report_received_) {
    RCLCPP_INFO(get_logger(), "   Publisher CPU:     Mean: %.1f%% | Max: %.1f%%",
      pub_cpu_mean_pct_, pub_cpu_max_pct_);
  } else {
    RCLCPP_INFO(get_logger(), "   Publisher CPU:     N/A");
  }
  RCLCPP_INFO(get_logger(), "   Subscriber CPU:    Mean: %.1f%% | Max: %.1f%%",
    sys_summary.process_cpu_mean_pct, sys_summary.process_cpu_max_pct);
  if (pub_report_received_) {
    RCLCPP_INFO(get_logger(), "   Publisher RSS:     Mean: %.1f MB | Peak: %.1f MB",
      pub_rss_mean_mb_, pub_rss_peak_mb_);
  } else {
    RCLCPP_INFO(get_logger(), "   Publisher RSS:     N/A");
  }
  RCLCPP_INFO(get_logger(), "   Subscriber RSS:    Mean: %.1f MB | Peak: %.1f MB",
    sys_summary.process_rss_mean_mb, sys_summary.process_rss_peak_mb);
  RCLCPP_INFO(get_logger(), "   System RAM Used:   %.1f MB",
    sys_summary.system_ram_mean_mb);
  if (sys_summary.gpu_available) {
    RCLCPP_INFO(get_logger(), "   GPU Utilization:   %.1f%%",
      sys_summary.gpu_util_mean_pct);
  } else {
    RCLCPP_INFO(get_logger(), "   GPU Utilization:   N/A");
  }
  RCLCPP_INFO(get_logger(), "==========================================================================");

  if (!summary_csv_path_.empty()) {
    std::ifstream check_file(summary_csv_path_);
    const bool exists = check_file.good();
    check_file.close();

    std::ofstream csv(summary_csv_path_, std::ios::app);
    if (csv.is_open()) {
      if (!exists) {
        csv << "backend,size_bytes,resolution,target_hz,actual_pub_fps,actual_recv_fps,"
            << "total_published_frames,total_received_frames,dropped_frames,frame_loss_pct,"
            << "cb_mean_latency_ms,cb_min_latency_ms,cb_p50_latency_ms,cb_p95_latency_ms,cb_p99_latency_ms,cb_max_latency_ms,cb_std_dev_ms,cb_outliers_3sigma_count,"
            << "br_mean_latency_ms,br_min_latency_ms,br_p50_latency_ms,br_p95_latency_ms,br_p99_latency_ms,br_max_latency_ms,br_std_dev_ms,br_outliers_3sigma_count,"
            << "pub_cpu_mean_pct,pub_cpu_max_pct,sub_cpu_mean_pct,sub_cpu_max_pct,"
            << "pub_rss_peak_mb,sub_rss_peak_mb,system_ram_mb,gpu_util_pct\n";
      }
      csv << backend << ","
          << size_bytes << ","
          << width << "x" << height << ","
          << std::fixed << std::setprecision(1) << target_hz_ << ","
          << std::fixed << std::setprecision(2) << actual_pub_hz << ","
          << std::fixed << std::setprecision(2) << actual_recv_hz << ","
          << total_published << ","
          << measured_frames << ","
          << lost_frames << ","
          << std::fixed << std::setprecision(2) << frame_loss_pct << ","
          << std::fixed << std::setprecision(3) << cb_stats.mean_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.min_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.p50_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.p95_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.p99_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.max_ms << ","
          << std::fixed << std::setprecision(3) << cb_stats.std_dev_ms << ","
          << cb_stats.outliers_count << ","
          << std::fixed << std::setprecision(3) << br_stats.mean_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.min_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.p50_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.p95_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.p99_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.max_ms << ","
          << std::fixed << std::setprecision(3) << br_stats.std_dev_ms << ","
          << br_stats.outliers_count << ",";

      if (pub_report_received_) {
        csv << std::fixed << std::setprecision(1) << pub_cpu_mean_pct_ << ","
            << std::fixed << std::setprecision(1) << pub_cpu_max_pct_ << ",";
      } else {
        csv << "N/A,N/A,";
      }

      csv << std::fixed << std::setprecision(1) << sys_summary.process_cpu_mean_pct << ","
          << std::fixed << std::setprecision(1) << sys_summary.process_cpu_max_pct << ",";

      if (pub_report_received_) {
        csv << std::fixed << std::setprecision(1) << pub_rss_peak_mb_ << ",";
      } else {
        csv << "N/A,";
      }

      csv << std::fixed << std::setprecision(1) << sys_summary.process_rss_peak_mb << ","
          << std::fixed << std::setprecision(1) << sys_summary.system_ram_mean_mb << ",";

      if (sys_summary.gpu_available) {
        csv << std::fixed << std::setprecision(1) << sys_summary.gpu_util_mean_pct << "\n";
      } else {
        csv << "N/A\n";
      }
    }
  }

  if (!latency_csv_path_.empty()) {
    std::ofstream csv(latency_csv_path_);
    if (csv.is_open()) {
      csv << "frame,size_bytes,callback_latency_ms,buffer_ready_latency_ms\n";
      for (std::size_t i = 0; i < callback_latencies_ms_.size(); ++i) {
        csv << (i + 1) << "," << size_bytes << ","
            << std::fixed << std::setprecision(3) << callback_latencies_ms_[i] << ","
            << std::fixed << std::setprecision(3) << buffer_ready_latencies_ms_[i] << "\n";
      }
    }
  }
}

}  // namespace cuda_bb_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(cuda_bb_benchmark::CudaImageSubscriberNode)
