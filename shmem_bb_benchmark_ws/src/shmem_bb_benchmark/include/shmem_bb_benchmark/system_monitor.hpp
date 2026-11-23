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

#ifndef SHMEM_BB_BENCHMARK__SYSTEM_MONITOR_HPP_
#define SHMEM_BB_BENCHMARK__SYSTEM_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace shmem_bb_benchmark
{

struct SystemMetricsSample
{
  double cpu_util_pct{0.0};
  double process_rss_mb{0.0};
  double system_ram_used_mb{0.0};
  double cpu_clock_avg_mhz{0.0};
  double cpu_clock_max_mhz{0.0};
  double power_watts{0.0};
  bool power_valid{false};
  double gpu_util_pct{0.0};
  double gpu_clock_mhz{0.0};
  double gpu_power_watts{0.0};
  bool gpu_valid{false};
};

struct SystemMetricsSummary
{
  double cpu_util_mean_pct{0.0};
  double cpu_util_max_pct{0.0};
  double process_rss_mean_mb{0.0};
  double process_rss_peak_mb{0.0};
  double system_ram_mean_mb{0.0};
  double cpu_clock_mean_mhz{0.0};
  double cpu_clock_max_mhz{0.0};
  double power_mean_watts{0.0};
  double power_max_watts{0.0};
  bool power_available{false};
  double gpu_util_mean_pct{0.0};
  double gpu_clock_mean_mhz{0.0};
  double gpu_power_mean_watts{0.0};
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
      return summary;
    }

    double sum_cpu = 0.0, max_cpu = 0.0;
    double sum_rss = 0.0, peak_rss = 0.0;
    double sum_sys_ram = 0.0;
    double sum_clock = 0.0, max_clock = 0.0;
    double sum_power = 0.0, max_power = 0.0;
    std::size_t power_count = 0;
    double sum_gpu_util = 0.0, sum_gpu_clock = 0.0, sum_gpu_power = 0.0;
    std::size_t gpu_count = 0;

    for (const auto & s : samples_) {
      sum_cpu += s.cpu_util_pct;
      max_cpu = std::max(max_cpu, s.cpu_util_pct);

      sum_rss += s.process_rss_mb;
      peak_rss = std::max(peak_rss, s.process_rss_mb);

      sum_sys_ram += s.system_ram_used_mb;

      sum_clock += s.cpu_clock_avg_mhz;
      max_clock = std::max(max_clock, s.cpu_clock_max_mhz);

      if (s.power_valid) {
        sum_power += s.power_watts;
        max_power = std::max(max_power, s.power_watts);
        ++power_count;
      }

      if (s.gpu_valid) {
        sum_gpu_util += s.gpu_util_pct;
        sum_gpu_clock += s.gpu_clock_mhz;
        sum_gpu_power += s.gpu_power_watts;
        ++gpu_count;
      }
    }

    const double n = static_cast<double>(samples_.size());
    summary.cpu_util_mean_pct = sum_cpu / n;
    summary.cpu_util_max_pct = max_cpu;
    summary.process_rss_mean_mb = sum_rss / n;
    summary.process_rss_peak_mb = peak_rss;
    summary.system_ram_mean_mb = sum_sys_ram / n;
    summary.cpu_clock_mean_mhz = sum_clock / n;
    summary.cpu_clock_max_mhz = max_clock;

    if (power_count > 0) {
      summary.power_available = true;
      summary.power_mean_watts = sum_power / static_cast<double>(power_count);
      summary.power_max_watts = max_power;
    }

    if (gpu_count > 0) {
      summary.gpu_available = true;
      summary.gpu_util_mean_pct = sum_gpu_util / static_cast<double>(gpu_count);
      summary.gpu_clock_mean_mhz = sum_gpu_clock / static_cast<double>(gpu_count);
      summary.gpu_power_mean_watts = sum_gpu_power / static_cast<double>(gpu_count);
    }

    return summary;
  }

private:
  void init_baseline()
  {
    read_cpu_ticks(prev_idle_ticks_, prev_total_ticks_);
    prev_time_ = std::chrono::steady_clock::now();
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

    // 1. CPU Utilization
    uint64_t idle_ticks = 0, total_ticks = 0;
    read_cpu_ticks(idle_ticks, total_ticks);
    const uint64_t delta_total = (total_ticks >= prev_total_ticks_) ? (total_ticks - prev_total_ticks_) : 0;
    const uint64_t delta_idle = (idle_ticks >= prev_idle_ticks_) ? (idle_ticks - prev_idle_ticks_) : 0;
    prev_total_ticks_ = total_ticks;
    prev_idle_ticks_ = idle_ticks;

    if (delta_total > 0) {
      sample.cpu_util_pct = (1.0 - static_cast<double>(delta_idle) / static_cast<double>(delta_total)) * 100.0;
    }

    // 2. RAM (System and Process)
    sample.system_ram_used_mb = read_system_ram_used_mb();
    sample.process_rss_mb = read_process_rss_mb();

    // 3. CPU Clocks
    read_cpu_clocks(sample.cpu_clock_avg_mhz, sample.cpu_clock_max_mhz);

    // 4. Power Draw
    sample.power_valid = read_power_watts(sample.power_watts);

    return sample;
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

  static double read_process_rss_mb()
  {
    std::ifstream file("/proc/self/status");
    if (!file.is_open()) return 0.0;

    std::string line;
    while (std::getline(file, line)) {
      if (line.rfind("VmRSS:", 0) == 0) {
        std::istringstream ss(line.substr(6));
        uint64_t rss_kb = 0;
        ss >> rss_kb;
        return static_cast<double>(rss_kb) / 1024.0;
      }
    }
    return 0.0;
  }

  static void read_cpu_clocks(double & avg_mhz, double & max_mhz)
  {
    std::vector<double> freqs;
    // Check /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
    for (int cpu = 0; cpu < 256; ++cpu) {
      std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/scaling_cur_freq";
      std::ifstream file(path);
      if (!file.is_open()) {
        if (cpu > 0) break;
        continue;
      }
      uint64_t khz = 0;
      if (file >> khz && khz > 0) {
        freqs.push_back(static_cast<double>(khz) / 1000.0);
      }
    }

    if (freqs.empty()) {
      // Fallback: /proc/cpuinfo
      std::ifstream file("/proc/cpuinfo");
      std::string line;
      while (std::getline(file, line)) {
        if (line.rfind("cpu MHz", 0) == 0) {
          auto colon = line.find(':');
          if (colon != std::string::npos) {
            std::istringstream ss(line.substr(colon + 1));
            double mhz = 0.0;
            if (ss >> mhz) {
              freqs.push_back(mhz);
            }
          }
        }
      }
    }

    if (!freqs.empty()) {
      double sum = 0.0, max_f = 0.0;
      for (double f : freqs) {
        sum += f;
        max_f = std::max(max_f, f);
      }
      avg_mhz = sum / static_cast<double>(freqs.size());
      max_mhz = max_f;
    } else {
      avg_mhz = 0.0;
      max_mhz = 0.0;
    }
  }

  static bool read_power_watts(double & power_w)
  {
    // 1. Direct power_now in /sys/class/power_supply/*/power_now (micro-Watts)
    DIR * dir = opendir("/sys/class/power_supply");
    if (dir != nullptr) {
      struct dirent * entry;
      while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        std::string p_path = std::string("/sys/class/power_supply/") + entry->d_name + "/power_now";
        std::ifstream file(p_path);
        uint64_t uw = 0;
        if (file.is_open() && file >> uw && uw > 0) {
          power_w = static_cast<double>(uw) / 1e6;
          closedir(dir);
          return true;
        }
      }
      closedir(dir);
    }

    // 2. Voltage * Current in /sys/class/power_supply/*/uevent
    dir = opendir("/sys/class/power_supply");
    if (dir != nullptr) {
      struct dirent * entry;
      while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        std::string u_path = std::string("/sys/class/power_supply/") + entry->d_name + "/uevent";
        std::ifstream file(u_path);
        if (file.is_open()) {
          std::string line;
          double volt_v = 0.0, curr_a = 0.0;
          while (std::getline(file, line)) {
            if (line.rfind("POWER_SUPPLY_VOLTAGE_NOW=", 0) == 0) {
              volt_v = std::stod(line.substr(25)) / 1e6;
            } else if (line.rfind("POWER_SUPPLY_CURRENT_NOW=", 0) == 0) {
              curr_a = std::stod(line.substr(25)) / 1e6;
            }
          }
          if (volt_v > 0.0 && curr_a > 0.0) {
            power_w = volt_v * curr_a;
            closedir(dir);
            return true;
          }
        }
      }
      closedir(dir);
    }

    // 3. Jetson INA3221 rail monitoring
    std::ifstream jetson_curr("/sys/bus/i2c/drivers/ina3221x/1-0040/hwmon/hwmon1/curr1_input");
    std::ifstream jetson_volt("/sys/bus/i2c/drivers/ina3221x/1-0040/hwmon/hwmon1/in1_input");
    if (jetson_curr.is_open() && jetson_volt.is_open()) {
      double ma = 0.0, mv = 0.0;
      if (jetson_curr >> ma && jetson_volt >> mv) {
        power_w = (ma / 1000.0) * (mv / 1000.0);
        return true;
      }
    }

    return false;
  }

  std::atomic<bool> running_{false};
  std::chrono::milliseconds sample_interval_{200};
  std::thread worker_thread_;
  mutable std::mutex mutex_;
  std::vector<SystemMetricsSample> samples_;

  uint64_t prev_idle_ticks_{0};
  uint64_t prev_total_ticks_{0};
  std::chrono::steady_clock::time_point prev_time_{};
};

}  // namespace shmem_bb_benchmark

#endif  // SHMEM_BB_BENCHMARK__SYSTEM_MONITOR_HPP_
