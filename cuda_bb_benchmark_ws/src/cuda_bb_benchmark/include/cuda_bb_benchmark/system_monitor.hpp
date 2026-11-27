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

#ifndef CUDA_BB_BENCHMARK__SYSTEM_MONITOR_HPP_
#define CUDA_BB_BENCHMARK__SYSTEM_MONITOR_HPP_

#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace cuda_bb_benchmark
{

struct SystemMetricsSample
{
  double process_cpu_pct{0.0};
  double process_pss_mb{0.0};
  bool pss_valid{false};
  double gpu_util_pct{0.0};
  double gpu_freq_mhz{0.0};
  bool gpu_valid{false};
  bool gpu_freq_valid{false};
  double emc_util_pct{0.0};
  double emc_freq_mhz{0.0};
  bool emc_valid{false};
};

struct SystemMetricsSummary
{
  double process_cpu_mean_pct{0.0};
  double process_cpu_max_pct{0.0};
  bool cpu_available{false};
  double process_pss_mean_mb{0.0};
  double process_pss_peak_mb{0.0};
  bool pss_available{false};
  double gpu_util_mean_pct{0.0};
  double gpu_freq_mean_mhz{0.0};
  bool gpu_available{false};
  bool gpu_freq_available{false};
  double emc_util_mean_pct{0.0};
  double emc_freq_mean_mhz{0.0};
  bool emc_available{false};
};

class SystemMonitor
{
public:
  SystemMonitor() = default;
  ~SystemMonitor()
  {
    stop();
  }

  void start(
    std::chrono::milliseconds interval = std::chrono::milliseconds(200),
    bool enable_gpu_telemetry = false)
  {
    if (running_.exchange(true)) {
      return;
    }
    sample_interval_ = interval;
    enable_gpu_telemetry_ = enable_gpu_telemetry;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      samples_.clear();
    }
    init_baseline();

    worker_thread_ = std::thread([this]() {
      while (running_.load()) {
        std::this_thread::sleep_for(sample_interval_);
        if (!running_.load()) {
          break;
        }
        auto sample = collect_single_sample();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          samples_.push_back(sample);
        }
      }
    });
  }

  void stop()
  {
    if (running_.exchange(false)) {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
    }
  }

  static long get_online_cpu_count()
  {
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0) ? n : 1L;
  }

  static bool read_environment_ram_mb(double & total_ram_mb, double & avail_ram_mb)
  {
    total_ram_mb = 0.0;
    avail_ram_mb = 0.0;
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) return false;

    std::string line;
    uint64_t mem_total_kb = 0, mem_avail_kb = 0;
    bool got_total = false, got_avail = false;
    while (std::getline(file, line)) {
      if (line.rfind("MemTotal:", 0) == 0) {
        std::istringstream ss(line.substr(9));
        if (ss >> mem_total_kb) got_total = true;
      } else if (line.rfind("MemAvailable:", 0) == 0) {
        std::istringstream ss(line.substr(13));
        if (ss >> mem_avail_kb) got_avail = true;
      }
      if (got_total && got_avail) break;
    }
    if (got_total) total_ram_mb = static_cast<double>(mem_total_kb) / 1024.0;
    if (got_avail) avail_ram_mb = static_cast<double>(mem_avail_kb) / 1024.0;
    return got_total;
  }

  SystemMetricsSummary get_summary() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SystemMetricsSummary summary;
    if (samples_.empty()) {
      double pss_mb = 0.0;
      if (read_process_pss_mb(pss_mb)) {
        summary.pss_available = true;
        summary.process_pss_mean_mb = pss_mb;
        summary.process_pss_peak_mb = pss_mb;
      }
      return summary;
    }

    double sum_cpu = 0.0, max_cpu = 0.0;
    double sum_pss = 0.0, peak_pss = 0.0;
    std::size_t pss_count = 0;
    double sum_gpu = 0.0;
    std::size_t gpu_count = 0;
    double sum_gpu_freq = 0.0;
    std::size_t gpu_freq_count = 0;
    double sum_emc_util = 0.0;
    double sum_emc_freq = 0.0;
    std::size_t emc_count = 0;

    for (const auto & s : samples_) {
      sum_cpu += s.process_cpu_pct;
      max_cpu = std::max(max_cpu, s.process_cpu_pct);

      if (s.pss_valid) {
        sum_pss += s.process_pss_mb;
        peak_pss = std::max(peak_pss, s.process_pss_mb);
        ++pss_count;
      }

      if (s.gpu_valid) {
        sum_gpu += s.gpu_util_pct;
        ++gpu_count;
      }
      if (s.gpu_freq_valid) {
        sum_gpu_freq += s.gpu_freq_mhz;
        ++gpu_freq_count;
      }
      if (s.emc_valid) {
        sum_emc_util += s.emc_util_pct;
        sum_emc_freq += s.emc_freq_mhz;
        ++emc_count;
      }
    }

    const double n = static_cast<double>(samples_.size());
    summary.cpu_available = true;
    summary.process_cpu_mean_pct = sum_cpu / n;
    summary.process_cpu_max_pct = max_cpu;

    if (pss_count > 0) {
      summary.pss_available = true;
      summary.process_pss_mean_mb = sum_pss / static_cast<double>(pss_count);
      summary.process_pss_peak_mb = peak_pss;
    }

    if (gpu_count > 0) {
      summary.gpu_available = true;
      summary.gpu_util_mean_pct = sum_gpu / static_cast<double>(gpu_count);
    }
    if (gpu_freq_count > 0) {
      summary.gpu_freq_available = true;
      summary.gpu_freq_mean_mhz = sum_gpu_freq / static_cast<double>(gpu_freq_count);
    }
    if (emc_count > 0) {
      summary.emc_available = true;
      summary.emc_util_mean_pct = sum_emc_util / static_cast<double>(emc_count);
      summary.emc_freq_mean_mhz = sum_emc_freq / static_cast<double>(emc_count);
    }

    return summary;
  }

private:
  void init_baseline()
  {
    read_process_cpu_ticks(prev_proc_ticks_);
    prev_wall_time_ = std::chrono::steady_clock::now();
    last_pss_sample_time_ = prev_wall_time_ - std::chrono::seconds(2);  // sample PSS on first tick
    clk_tck_ = sysconf(_SC_CLK_TCK);
    if (clk_tck_ <= 0) clk_tck_ = 100;
    num_cpus_ = get_online_cpu_count();
  }

  static bool read_process_cpu_ticks(uint64_t & proc_ticks)
  {
    proc_ticks = 0;
    std::ifstream file("/proc/self/stat");
    if (!file.is_open()) return false;
    std::string line;
    if (!std::getline(file, line)) return false;
    auto last_paren = line.rfind(')');
    if (last_paren == std::string::npos || last_paren + 2 >= line.size()) return false;
    std::istringstream ss(line.substr(last_paren + 2));
    char state;
    int ppid, pgrp, session, tty_nr, tpgid;
    unsigned int flags;
    uint64_t minflt, cminflt, majflt, cmajflt, utime, stime;
    if (ss >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags
           >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime) {
      proc_ticks = utime + stime;
      return true;
    }
    return false;
  }

  static bool read_process_pss_mb(double & pss_mb)
  {
    pss_mb = 0.0;
    std::ifstream file("/proc/self/smaps_rollup");
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
      if (line.rfind("Pss:", 0) == 0) {
        std::istringstream ss(line.substr(4));
        uint64_t pss_kb = 0;
        if (ss >> pss_kb) {
          pss_mb = static_cast<double>(pss_kb) / 1024.0;
          return true;
        }
      }
    }
    return false;
  }

  SystemMetricsSample collect_single_sample()
  {
    SystemMetricsSample sample;
    const auto now = std::chrono::steady_clock::now();

    // 1. Process CPU Utilization: (process CPU time / elapsed wall-clock time) * 100 / online CPUs
    uint64_t proc_ticks = 0;
    read_process_cpu_ticks(proc_ticks);
    const uint64_t delta_proc = (proc_ticks >= prev_proc_ticks_) ? (proc_ticks - prev_proc_ticks_) : 0;
    prev_proc_ticks_ = proc_ticks;

    const double elapsed_wall_sec = std::chrono::duration<double>(now - prev_wall_time_).count();
    prev_wall_time_ = now;

    if (elapsed_wall_sec > 0.0) {
      const double proc_cpu_sec = static_cast<double>(delta_proc) / static_cast<double>(clk_tck_);
      sample.process_cpu_pct =
        (proc_cpu_sec / elapsed_wall_sec) * 100.0 / static_cast<double>(num_cpus_);
    }

    // 2. Process PSS (~1s sampling period via /proc/self/smaps_rollup)
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pss_sample_time_).count() >= 1000) {
      sample.pss_valid = read_process_pss_mb(sample.process_pss_mb);
      last_pss_sample_time_ = now;
    }

    // 3. Optional GPU / EMC Telemetry (enabled only when requested)
    if (enable_gpu_telemetry_) {
      collect_hardware_telemetry(sample);
    }

    return sample;
  }

  static void collect_hardware_telemetry(SystemMetricsSample & sample)
  {
    static bool checked_env = false;
    static bool has_tegrastats = false;
    static bool has_nvidia_smi = false;

    if (!checked_env) {
      checked_env = true;
      has_tegrastats = (std::system("which tegrastats >/dev/null 2>&1") == 0);
      if (!has_tegrastats) {
        has_nvidia_smi = (std::system("which nvidia-smi >/dev/null 2>&1") == 0);
      }
    }

    if (has_tegrastats) {
      if (read_tegrastats_sample(sample)) {
        return;
      }
    }

    read_sysfs_jetson_telemetry(sample);

    if (!sample.gpu_valid && has_nvidia_smi) {
      sample.gpu_valid = read_nvidia_smi_utilization(sample.gpu_util_pct);
    }
  }

  static bool read_tegrastats_sample(SystemMetricsSample & sample)
  {
    FILE * pipe = popen("timeout 1s tegrastats --interval 100 2>/dev/null | head -n 1", "r");
    if (!pipe) return false;
    char buffer[1024];
    bool read_ok = (fgets(buffer, sizeof(buffer), pipe) != nullptr);
    pclose(pipe);
    if (!read_ok) return false;

    parse_tegrastats_line(std::string(buffer), sample);
    return true;
  }

  static void parse_tegrastats_line(const std::string & text, SystemMetricsSample & sample)
  {
    auto pos_gr3d = text.find("GR3D_FREQ");
    if (pos_gr3d != std::string::npos) {
      auto start = pos_gr3d + 9;
      while (start < text.size() && text[start] == ' ') ++start;
      auto pct_pos = text.find('%', start);
      auto at_pos = text.find('@', start);
      if (pct_pos != std::string::npos && (at_pos == std::string::npos || pct_pos < at_pos)) {
        try {
          sample.gpu_util_pct = std::stod(text.substr(start, pct_pos - start));
          sample.gpu_valid = true;
        } catch (...) {}
      }
      if (at_pos != std::string::npos) {
        try {
          auto end = at_pos + 1;
          while (end < text.size() && (std::isdigit(text[end]) || text[end] == '.')) ++end;
          sample.gpu_freq_mhz = std::stod(text.substr(at_pos + 1, end - (at_pos + 1)));
          sample.gpu_freq_valid = true;
        } catch (...) {}
      }
    }

    auto pos_emc = text.find("EMC_FREQ");
    if (pos_emc != std::string::npos) {
      auto start = pos_emc + 8;
      while (start < text.size() && text[start] == ' ') ++start;
      auto pct_pos = text.find('%', start);
      auto at_pos = text.find('@', start);
      if (pct_pos != std::string::npos && at_pos != std::string::npos && pct_pos < at_pos) {
        try {
          sample.emc_util_pct = std::stod(text.substr(start, pct_pos - start));
          auto end = at_pos + 1;
          while (end < text.size() && (std::isdigit(text[end]) || text[end] == '.')) ++end;
          sample.emc_freq_mhz = std::stod(text.substr(at_pos + 1, end - (at_pos + 1)));
          sample.emc_valid = true;
        } catch (...) {}
      }
    }
  }

  static void read_sysfs_jetson_telemetry(SystemMetricsSample & sample)
  {
    static const std::vector<std::string> gpu_load_paths = {
      "/sys/devices/gpu.0/load",
      "/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/device/load",
      "/sys/devices/platform/17000000.ga10b/devfreq/17000000.ga10b/device/load"
    };
    for (const auto & path : gpu_load_paths) {
      std::ifstream f(path);
      if (f.is_open()) {
        double val = 0.0;
        if (f >> val) {
          if (val > 100.0) val /= 10.0;
          sample.gpu_util_pct = val;
          sample.gpu_valid = true;
          break;
        }
      }
    }

    static const std::vector<std::string> gpu_freq_paths = {
      "/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/cur_freq",
      "/sys/devices/platform/17000000.ga10b/devfreq/17000000.ga10b/cur_freq",
      "/sys/devices/gpu.0/devfreq/17000000.gpu/cur_freq"
    };
    for (const auto & path : gpu_freq_paths) {
      std::ifstream f(path);
      if (f.is_open()) {
        double hz = 0.0;
        if (f >> hz) {
          sample.gpu_freq_mhz = hz > 1e6 ? hz / 1e6 : (hz > 1e3 ? hz / 1e3 : hz);
          sample.gpu_freq_valid = true;
          break;
        }
      }
    }

    static const std::vector<std::string> emc_freq_paths = {
      "/sys/kernel/debug/clk/emc/clk_rate",
      "/sys/devices/platform/17000010.devfreq-emc/devfreq/17000010.devfreq-emc/cur_freq"
    };
    for (const auto & path : emc_freq_paths) {
      std::ifstream f(path);
      if (f.is_open()) {
        double hz = 0.0;
        if (f >> hz) {
          sample.emc_freq_mhz = hz > 1e6 ? hz / 1e6 : (hz > 1e3 ? hz / 1e3 : hz);
          sample.emc_valid = true;
          break;
        }
      }
    }
  }

  static bool read_nvidia_smi_utilization(double & gpu_util_pct)
  {
    gpu_util_pct = 0.0;
    FILE * pipe = popen("nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null", "r");
    if (!pipe) return false;
    char buffer[64];
    bool ok = false;
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      std::istringstream ss(buffer);
      if (ss >> gpu_util_pct) {
        ok = true;
      }
    }
    pclose(pipe);
    return ok;
  }

  std::atomic<bool> running_{false};
  std::chrono::milliseconds sample_interval_{200};
  bool enable_gpu_telemetry_{false};
  mutable std::mutex mutex_;
  std::vector<SystemMetricsSample> samples_;
  std::thread worker_thread_;

  uint64_t prev_proc_ticks_{0};
  std::chrono::steady_clock::time_point prev_wall_time_{};
  std::chrono::steady_clock::time_point last_pss_sample_time_{};
  long clk_tck_{100};
  long num_cpus_{1};
};

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__SYSTEM_MONITOR_HPP_
