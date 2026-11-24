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
#include <iostream>
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
  double process_rss_mb{0.0};
  double process_rss_peak_mb{0.0};
  double system_ram_used_mb{0.0};
  double gpu_util_pct{0.0};
  bool gpu_valid{false};
};

struct SystemMetricsSummary
{
  double process_cpu_mean_pct{0.0};
  double process_cpu_max_pct{0.0};
  double process_rss_mean_mb{0.0};
  double process_rss_peak_mb{0.0};
  double system_ram_mean_mb{0.0};
  double gpu_util_mean_pct{0.0};
  bool gpu_available{false};
};

class SystemMonitor
{
public:
  SystemMonitor() = default;
  ~SystemMonitor()
  {
    stop();
  }

  void start(std::chrono::milliseconds interval = std::chrono::milliseconds(200))
  {
    if (running_.exchange(true)) {
      return;
    }
    sample_interval_ = interval;
    samples_.clear();
    init_baseline();

    worker_thread_ = std::thread([this]() {
      while (running_.load()) {
        auto sample = collect_single_sample();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          samples_.push_back(sample);
        }
        std::this_thread::sleep_for(sample_interval_);
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

  SystemMetricsSummary get_summary() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SystemMetricsSummary summary;
    if (samples_.empty()) {
      double rss = 0.0, peak = 0.0;
      read_process_rss_mb(rss, peak);
      summary.process_rss_mean_mb = rss;
      summary.process_rss_peak_mb = peak;
      summary.system_ram_mean_mb = read_system_ram_used_mb();
      return summary;
    }

    double sum_cpu = 0.0, max_cpu = 0.0;
    double sum_rss = 0.0, peak_rss = 0.0;
    double sum_sys_ram = 0.0;
    double sum_gpu = 0.0;
    std::size_t gpu_count = 0;

    for (const auto & s : samples_) {
      sum_cpu += s.process_cpu_pct;
      max_cpu = std::max(max_cpu, s.process_cpu_pct);

      sum_rss += s.process_rss_mb;
      peak_rss = std::max(peak_rss, s.process_rss_peak_mb);

      sum_sys_ram += s.system_ram_used_mb;

      if (s.gpu_valid) {
        sum_gpu += s.gpu_util_pct;
        ++gpu_count;
      }
    }

    const double n = static_cast<double>(samples_.size());
    summary.process_cpu_mean_pct = sum_cpu / n;
    summary.process_cpu_max_pct = max_cpu;
    summary.process_rss_mean_mb = sum_rss / n;
    summary.process_rss_peak_mb = peak_rss;
    summary.system_ram_mean_mb = sum_sys_ram / n;

    if (gpu_count > 0) {
      summary.gpu_available = true;
      summary.gpu_util_mean_pct = sum_gpu / static_cast<double>(gpu_count);
    }

    return summary;
  }

private:
  void init_baseline()
  {
    read_process_cpu_ticks(prev_proc_ticks_);
    uint64_t idle = 0;
    read_cpu_ticks(idle, prev_total_ticks_);
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

  static void read_cpu_ticks(uint64_t & idle, uint64_t & total)
  {
    idle = 0;
    total = 0;
    std::ifstream file("/proc/stat");
    if (!file.is_open()) return;

    std::string line;
    if (std::getline(file, line) && line.rfind("cpu ", 0) == 0) {
      std::istringstream ss(line.substr(4));
      uint64_t user, nice, system, idle_t, iowait, irq, softirq, steal;
      if (ss >> user >> nice >> system >> idle_t >> iowait >> irq >> softirq >> steal) {
        idle = idle_t + iowait;
        total = user + nice + system + idle_t + iowait + irq + softirq + steal;
      }
    }
  }

  SystemMetricsSample collect_single_sample()
  {
    SystemMetricsSample sample;

    // 1. Process CPU Utilization
    uint64_t proc_ticks = 0;
    read_process_cpu_ticks(proc_ticks);
    const uint64_t delta_proc = (proc_ticks >= prev_proc_ticks_) ? (proc_ticks - prev_proc_ticks_) : 0;
    prev_proc_ticks_ = proc_ticks;

    uint64_t idle_ticks = 0, total_ticks = 0;
    read_cpu_ticks(idle_ticks, total_ticks);
    const uint64_t delta_total = (total_ticks >= prev_total_ticks_) ? (total_ticks - prev_total_ticks_) : 0;
    prev_total_ticks_ = total_ticks;

    if (delta_total > 0) {
      long num_cpus = sysconf(_SC_NPROCESSORS_ONLN);
      if (num_cpus <= 0) num_cpus = 1;
      sample.process_cpu_pct = 100.0 * (static_cast<double>(delta_proc) / static_cast<double>(delta_total)) * static_cast<double>(num_cpus);
    }

    // 2. RAM (Process RSS and System RAM)
    read_process_rss_mb(sample.process_rss_mb, sample.process_rss_peak_mb);
    sample.system_ram_used_mb = read_system_ram_used_mb();

    // 3. GPU (only if reliably available)
    sample.gpu_valid = read_gpu_utilization(sample.gpu_util_pct);

    return sample;
  }

  static void read_process_rss_mb(double & current_rss_mb, double & peak_rss_mb)
  {
    current_rss_mb = 0.0;
    peak_rss_mb = 0.0;
    std::ifstream file("/proc/self/status");
    if (!file.is_open()) return;

    std::string line;
    while (std::getline(file, line)) {
      if (line.rfind("VmRSS:", 0) == 0) {
        std::istringstream ss(line.substr(6));
        uint64_t rss_kb = 0;
        ss >> rss_kb;
        current_rss_mb = static_cast<double>(rss_kb) / 1024.0;
      } else if (line.rfind("VmHWM:", 0) == 0) {
        std::istringstream ss(line.substr(6));
        uint64_t hwm_kb = 0;
        ss >> hwm_kb;
        peak_rss_mb = static_cast<double>(hwm_kb) / 1024.0;
      }
    }
  }

  static double read_system_ram_used_mb()
  {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) return 0.0;

    std::string line;
    uint64_t mem_total_kb = 0, mem_avail_kb = 0;
    while (std::getline(file, line)) {
      if (line.rfind("MemTotal:", 0) == 0) {
        std::istringstream ss(line.substr(9));
        ss >> mem_total_kb;
      } else if (line.rfind("MemAvailable:", 0) == 0) {
        std::istringstream ss(line.substr(13));
        ss >> mem_avail_kb;
      }
      if (mem_total_kb > 0 && mem_avail_kb > 0) break;
    }
    return (mem_total_kb >= mem_avail_kb) ?
      static_cast<double>(mem_total_kb - mem_avail_kb) / 1024.0 : 0.0;
  }

  static bool read_gpu_utilization(double & gpu_util_pct)
  {
    gpu_util_pct = 0.0;
    static bool checked_binary = false;
    static bool has_nvidia_smi = false;
    if (!checked_binary) {
      checked_binary = true;
      has_nvidia_smi = (std::system("which nvidia-smi >/dev/null 2>&1") == 0);
    }
    if (!has_nvidia_smi) {
      return false;
    }

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
  mutable std::mutex mutex_;
  std::vector<SystemMetricsSample> samples_;
  std::thread worker_thread_;

  uint64_t prev_proc_ticks_{0};
  uint64_t prev_total_ticks_{0};
};

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__SYSTEM_MONITOR_HPP_