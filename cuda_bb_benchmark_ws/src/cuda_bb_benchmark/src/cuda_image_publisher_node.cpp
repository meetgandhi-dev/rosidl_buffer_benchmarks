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

#include "cuda_bb_benchmark/cuda_image_publisher_node.hpp"

#include <chrono>
#include <cstring>
#include <thread>
#include <unistd.h>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_bb_benchmark/test_pattern_utils.hpp"

namespace cuda_bb_benchmark
{

CudaImagePublisherNode::CudaImagePublisherNode(const rclcpp::NodeOptions & options)
: Node("cuda_image_publisher", options)
{
  topic_name_ = declare_parameter<std::string>("topic", "test_cuda_image");
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
  use_cuda_ = declare_parameter<bool>("use_cuda", true);
  cuda_device_id_ = declare_parameter<int>("cuda_device_id", 0);
  summary_csv_path_ = declare_parameter<std::string>("summary_csv_path", "");

  if (use_cuda_) {
    cudaError_t err = cudaSetDevice(cuda_device_id_);
    if (err != cudaSuccess) {
      RCLCPP_FATAL(
        get_logger(), "cudaSetDevice(%d) failed during publisher initialization: %s",
        cuda_device_id_, cudaGetErrorString(err));
      throw std::runtime_error(std::string("cudaSetDevice failed: ") + cudaGetErrorString(err));
    }
    err = cudaStreamCreate(&stream_);
    if (err != cudaSuccess || stream_ == nullptr) {
      RCLCPP_FATAL(
        get_logger(), "cudaStreamCreate() failed during publisher initialization: %s",
        cudaGetErrorString(err));
      throw std::runtime_error(std::string("cudaStreamCreate failed: ") + cudaGetErrorString(err));
    }
  }

  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, history_depth_);
  count_publisher_ = create_publisher<std_msgs::msg::UInt32>(topic_name_ + "/pub_count", 10);
  pub_report_publisher_ = create_publisher<std_msgs::msg::String>(
    topic_name_ + "/pub_report", rclcpp::QoS(1).reliable().transient_local());

  const auto timer_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&CudaImagePublisherNode::publish_timer_callback, this));

  double total_ram_mb = 0.0, avail_ram_mb = 0.0;
  SystemMonitor::read_environment_ram_mb(total_ram_mb, avail_ram_mb);
  RCLCPP_INFO(
    get_logger(),
    "CudaImagePublisherNode initialized: topic='%s', rate=%.1f Hz, res=%dx%d, max_msgs=%d, warmup_msgs=%d, history_depth=%d, use_cuda=%s (online_cpus=%ld, host_ram=%.0f MB total / %.0f MB avail)",
    topic_name_.c_str(), publish_rate_hz_, image_width_, image_height_, max_messages_, warmup_messages_, history_depth_,
    use_cuda_ ? "true" : "false", SystemMonitor::get_online_cpu_count(), total_ram_mb, avail_ram_mb);
}

CudaImagePublisherNode::~CudaImagePublisherNode()
{
  if (stream_ != nullptr) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

void CudaImagePublisherNode::publish_timer_callback()
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
          // Safety watchdog: ensure publisher terminates and does not hang on DDS teardown
          std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            _exit(0);
          }).detach();
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
  const size_t bytes_per_pixel = (encoding_ == "rgb8" || encoding_ == "bgr8") ? 3u :
    ((encoding_ == "rgba8" || encoding_ == "bgra8") ? 4u : 1u);
  const std::size_t payload_size = static_cast<std::size_t>(image_width_) * image_height_ * bytes_per_pixel;

  sensor_msgs::msg::Image msg;
  // Construct frame_id before timestamp capture
  msg.header.frame_id = "frame_" + std::to_string(seq);
  msg.height = static_cast<std::uint32_t>(image_height_);
  msg.width = static_cast<std::uint32_t>(image_width_);
  msg.encoding = encoding_;
  msg.is_bigendian = 0;
  msg.step = static_cast<std::uint32_t>(image_width_ * bytes_per_pixel);

  const uint8_t pattern_byte = generate_sequence_byte(seq);

  if (use_cuda_) {
    // Allocate CUDA buffer through cuda_buffer_backend pool
    bool cuda_pub_ok = true;
    try {
      msg.data = cuda_buffer_backend::allocate_buffer(payload_size);
      auto write_handle = cuda_buffer_backend::from_output_buffer(msg.data, stream_);
      void * dev_ptr = write_handle.get_ptr();
      if (payload_size > 0 && dev_ptr == nullptr) {
        cuda_pub_ok = false;
        RCLCPP_ERROR(
          get_logger(),
          "CUDA output buffer pointer is null for non-empty payload (%zu B) on msg #%zu; skipping publish.",
          payload_size, seq);
      } else if (payload_size > 0) {
        cudaError_t err = cudaMemsetAsync(
          dev_ptr, static_cast<int>(pattern_byte), payload_size, stream_);
        if (err != cudaSuccess) {
          cuda_pub_ok = false;
          RCLCPP_ERROR(
            get_logger(), "cudaMemsetAsync failed on msg #%zu: %s; skipping publish.",
            seq, cudaGetErrorString(err));
        } else {
          err = cudaStreamSynchronize(stream_);
          if (err != cudaSuccess) {
            cuda_pub_ok = false;
            RCLCPP_ERROR(
              get_logger(), "cudaStreamSynchronize failed on msg #%zu: %s; skipping publish.",
              seq, cudaGetErrorString(err));
          }
        }
      }
    } catch (const std::exception & e) {
      cuda_pub_ok = false;
      RCLCPP_ERROR(
        get_logger(), "Exception preparing CUDA buffer on msg #%zu: %s; skipping publish.",
        seq, e.what());
    }

    if (!cuda_pub_ok) {
      ++failed_pub_ops_count_;
      return;
    }
  } else {
    // Non-CUDA CPU-backed transport: allocate and populate standard CPU buffer
    msg.data = rosidl::Buffer<std::uint8_t>(payload_size);
    std::memset(msg.data.data(), static_cast<int>(pattern_byte), payload_size);
  }

  // Keep recent published messages alive in pool ring buffer
  message_history_.push_back(std::move(msg));
  if (static_cast<int>(message_history_.size()) > history_depth_) {
    message_history_.pop_front();
  }

  // Capture publisher timestamp IMMEDIATELY before publish() to avoid counting
  // buffer preparation, allocation, or frame_id construction in transport latency
  const auto steady_now = std::chrono::steady_clock::now();
  const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();

  if (seq == static_cast<std::size_t>(warmup_messages_ + 1)) {
    benchmark_start_time_ = steady_now;
    proc_monitor_.start(std::chrono::milliseconds(200), true);
  }
  if (seq > static_cast<std::size_t>(warmup_messages_)) {
    benchmark_end_time_ = steady_now;
  }

  auto & pub_msg = message_history_.back();
  pub_msg.header.stamp.sec = static_cast<int32_t>(steady_ns / 1000000000LL);
  pub_msg.header.stamp.nanosec = static_cast<uint32_t>(steady_ns % 1000000000LL);

  publisher_->publish(pub_msg);
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

void CudaImagePublisherNode::write_publisher_summary(double actual_fps, std::size_t total_sent)
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
  RCLCPP_INFO(get_logger(), " Throughput:  Target: %.1f Hz | Actual Pub: %.2f FPS | Total Published: %zu | Failed Pub Ops: %zu",
    publish_rate_hz_, actual_fps, total_sent, failed_pub_ops_count_);
  RCLCPP_INFO(get_logger(), " System Resources:");
  if (sys_summary.cpu_available) {
    RCLCPP_INFO(get_logger(), "   Publisher CPU:     Mean: %.1f%% | Peak: %.1f%%",
      sys_summary.process_cpu_mean_pct, sys_summary.process_cpu_max_pct);
  } else {
    RCLCPP_INFO(get_logger(), "   Publisher CPU:     N/A");
  }
  if (sys_summary.pss_available) {
    RCLCPP_INFO(get_logger(), "   Publisher PSS:     Mean: %.1f MB | Peak: %.1f MB",
      sys_summary.process_pss_mean_mb, sys_summary.process_pss_peak_mb);
  } else {
    RCLCPP_INFO(get_logger(), "   Publisher PSS:     N/A");
  }
  RCLCPP_INFO(get_logger(), "==========================================================================");

  // Publish report to subscriber using reliable + transient_local QoS
  if (pub_report_publisher_) {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "actual_pub_fps=" << actual_fps
       << ";total_published=" << total_sent
       << ";pub_cpu_avail=" << (sys_summary.cpu_available ? 1 : 0)
       << ";pub_cpu_mean=" << sys_summary.process_cpu_mean_pct
       << ";pub_cpu_max=" << sys_summary.process_cpu_max_pct
       << ";pub_pss_avail=" << (sys_summary.pss_available ? 1 : 0)
       << ";pub_pss_mean=" << sys_summary.process_pss_mean_mb
       << ";pub_pss_peak=" << sys_summary.process_pss_peak_mb;
    msg.data = ss.str();
    pub_report_publisher_->publish(msg);
  }

  // Write CSV if path provided
  if (!summary_csv_path_.empty()) {
    std::ofstream csv(summary_csv_path_);
    if (csv.is_open()) {
      csv << "actual_pub_fps,total_published_frames,pub_cpu_mean_pct,pub_cpu_peak_pct,pub_pss_mean_mb,pub_pss_peak_mb\n"
          << std::fixed << std::setprecision(2) << actual_fps << ","
          << total_sent << ",";
      if (sys_summary.cpu_available) {
        csv << std::fixed << std::setprecision(1) << sys_summary.process_cpu_mean_pct << ","
            << std::fixed << std::setprecision(1) << sys_summary.process_cpu_max_pct << ",";
      } else {
        csv << "N/A,N/A,";
      }
      if (sys_summary.pss_available) {
        csv << std::fixed << std::setprecision(1) << sys_summary.process_pss_mean_mb << ","
            << std::fixed << std::setprecision(1) << sys_summary.process_pss_peak_mb << "\n";
      } else {
        csv << "N/A,N/A\n";
      }
    }
  }
}

}  // namespace cuda_bb_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(cuda_bb_benchmark::CudaImagePublisherNode)
