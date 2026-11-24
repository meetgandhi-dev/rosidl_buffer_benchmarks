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
  double gpu_freq_mhz{0.0};
  bool gpu_valid{false};
  bool gpu_freq_valid{false};
  double emc_util_pct{0.0};
  double emc_freq_mhz{0.0};
  bool emc_valid{false};
  double cpu_temp_c{0.0};
  bool cpu_temp_valid{false};
  double gpu_temp_c{0.0};
  bool gpu_temp_valid{false};
};

struct SystemMetricsSummary
{
  double process_cpu_mean_pct{0.0};
  double process_cpu_max_pct{0.0};
  double process_rss_mean_mb{0.0};
  double process_rss_peak_mb{0.0};
  double system_ram_mean_mb{0.0};
  double gpu_util_mean_pct{0.0};
  double gpu_freq_mean_mhz{0.0};
  bool gpu_available{false};
  bool gpu_freq_available{false};
  double emc_util_mean_pct{0.0};
  double emc_freq_mean_mhz{0.0};
  bool emc_available{false};
  double cpu_temp_mean_c{0.0};
  bool cpu_temp_available{false};
  double gpu_temp_mean_c{0.0};
  bool gpu_temp_available{false};
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
      // Fallback: collect at least one instantaneous sample
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
    double sum_gpu_freq = 0.0;
    std::size_t gpu_freq_count = 0;
    double sum_emc_util = 0.0;
    double sum_emc_freq = 0.0;
    std::size_t emc_count = 0;
    double sum_cpu_temp = 0.0;
    std::size_t cpu_temp_count = 0;
    double sum_gpu_temp = 0.0;
    std::size_t gpu_temp_count = 0;

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
      if (s.gpu_freq_valid) {
        sum_gpu_freq += s.gpu_freq_mhz;
        ++gpu_freq_count;
      }
      if (s.emc_valid) {
        sum_emc_util += s.emc_util_pct;
        sum_emc_freq += s.emc_freq_mhz;
        ++emc_count;
      }
      if (s.cpu_temp_valid) {
        sum_cpu_temp += s.cpu_temp_c;
        ++cpu_temp_count;
      }
      if (s.gpu_temp_valid) {
        sum_gpu_temp += s.gpu_temp_c;
        ++gpu_temp_count;
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
    if (gpu_freq_count > 0) {
      summary.gpu_freq_available = true;
      summary.gpu_freq_mean_mhz = sum_gpu_freq / static_cast<double>(gpu_freq_count);
    }
    if (emc_count > 0) {
      summary.emc_available = true;
      summary.emc_util_mean_pct = sum_emc_util / static_cast<double>(emc_count);
      summary.emc_freq_mean_mhz = sum_emc_freq / static_cast<double>(emc_count);
    }
    if (cpu_temp_count > 0) {
      summary.cpu_temp_available = true;
      summary.cpu_temp_mean_c = sum_cpu_temp / static_cast<double>(cpu_temp_count);
    }
    if (gpu_temp_count > 0) {
      summary.gpu_temp_available = true;
      summary.gpu_temp_mean_c = sum_gpu_temp / static_cast<double>(gpu_temp_count);
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

    // 3. Hardware Telemetry (Jetson tegrastats / sysfs, or desktop nvidia-smi / sysfs thermal)
    collect_hardware_telemetry(sample);

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
      // Query Jetson tegrastats
      if (read_tegrastats_sample(sample)) {
        return;
      }
    }

    // Try sysfs directly for Jetson devfreq / thermal
    read_sysfs_jetson_telemetry(sample);

    // If GPU still not valid, try desktop nvidia-smi
    if (!sample.gpu_valid && has_nvidia_smi) {
      sample.gpu_valid = read_nvidia_smi_utilization(sample.gpu_util_pct);
    }

    // If temperatures still not found, try generic thermal zones (e.g. x86 / Linux generic)
    if (!sample.cpu_temp_valid || !sample.gpu_temp_valid) {
      read_generic_thermal_zones(sample);
    }
  }

  static bool read_tegrastats_sample(SystemMetricsSample & sample)
  {
    FILE * pipe = popen("tegrastats --interval 100 2>/dev/null | head -n 1", "r");
    if (!pipe) return false;
    char buffer[1024];
    bool read_ok = (fgets(buffer, sizeof(buffer), pipe) != nullptr);
    pclose(pipe);
    if (!read_ok) return false;

    std::string text(buffer);
    parse_tegrastats_line(text, sample);
    return true;
  }

  static void parse_tegrastats_line(const std::string & text, SystemMetricsSample & sample)
  {
    // Parse GR3D_FREQ: e.g. "GR3D_FREQ 15%@1300" or "GR3D_FREQ 0%@..."
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

    // Parse EMC_FREQ: e.g. "EMC_FREQ 2%@3199" or "EMC_FREQ 0%@204"
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

    // Parse CPU / GPU temperatures: e.g. "cpu 38.5C", "gpu 37.6C", "CPU@38.5C", "GPU@37.6C"
    // Case-insensitive / common tegrastats tokens
    parse_temperature_token(text, "cpu", sample.cpu_temp_c, sample.cpu_temp_valid);
    if (!sample.cpu_temp_valid) {
      parse_temperature_token(text, "CPU", sample.cpu_temp_c, sample.cpu_temp_valid);
    }
    parse_temperature_token(text, "gpu", sample.gpu_temp_c, sample.gpu_temp_valid);
    if (!sample.gpu_temp_valid) {
      parse_temperature_token(text, "GPU", sample.gpu_temp_c, sample.gpu_temp_valid);
    }
  }

  static void parse_temperature_token(const std::string & text, const std::string & token, double & temp_c, bool & valid)
  {
    auto pos = text.find(token);
    while (pos != std::string::npos) {
      // Check word boundary before token
      if (pos == 0 || text[pos - 1] == ' ' || text[pos - 1] == '[' || text[pos - 1] == ',') {
        auto after = pos + token.size();
        if (after < text.size() && (text[after] == ' ' || text[after] == '@')) {
          while (after < text.size() && (text[after] == ' ' || text[after] == '@')) ++after;
          auto end = after;
          while (end < text.size() && (std::isdigit(text[end]) || text[end] == '.' || text[end] == '-')) ++end;
          if (end > after && end < text.size() && (text[end] == 'C' || text[end] == 'c')) {
            try {
              temp_c = std::stod(text.substr(after, end - after));
              valid = true;
              return;
            } catch (...) {}
          }
        }
      }
      pos = text.find(token, pos + 1);
    }
  }

  static void read_sysfs_jetson_telemetry(SystemMetricsSample & sample)
  {
    // Check GPU load from Jetson sysfs nodes
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
          // On some kernels load is 0-1000 (per mille), on others 0-100%
          if (val > 100.0) val /= 10.0;
          sample.gpu_util_pct = val;
          sample.gpu_valid = true;
          break;
        }
      }
    }

    // Check GPU frequency from Jetson devfreq
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

    // Check EMC frequency / utilization
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

  static void read_generic_thermal_zones(SystemMetricsSample & sample)
  {
    for (int zone = 0; zone < 20; ++zone) {
      std::string type_path = "/sys/class/thermal/thermal_zone" + std::to_string(zone) + "/type";
      std::string temp_path = "/sys/class/thermal/thermal_zone" + std::to_string(zone) + "/temp";
      std::ifstream type_file(type_path);
      if (!type_file.is_open()) break;
      std::string type_name;
      std::getline(type_file, type_name);

      std::ifstream temp_file(temp_path);
      if (!temp_file.is_open()) continue;
      double raw_temp = 0.0;
      if (!(temp_file >> raw_temp)) continue;
      double temp_c = (raw_temp > 1000.0) ? (raw_temp / 1000.0) : raw_temp;

      // Identify CPU thermal zone
      if (!sample.cpu_temp_valid) {
        if (type_name == "CPU-therm" || type_name == "cpu-therm" ||
            type_name == "TCPU" || type_name == "x86_pkg_temp" ||
            type_name.find("cpu") != std::string::npos ||
            type_name.find("CPU") != std::string::npos) {
          sample.cpu_temp_c = temp_c;
          sample.cpu_temp_valid = true;
        }
      }

      // Identify GPU thermal zone
      if (!sample.gpu_temp_valid) {
        if (type_name == "GPU-therm" || type_name == "gpu-therm" ||
            type_name.find("gpu") != std::string::npos ||
            type_name.find("GPU") != std::string::npos) {
          sample.gpu_temp_c = temp_c;
          sample.gpu_temp_valid = true;
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
  mutable std::mutex mutex_;
  std::vector<SystemMetricsSample> samples_;
  std::thread worker_thread_;

  uint64_t prev_proc_ticks_{0};
  uint64_t prev_total_ticks_{0};
};

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__SYSTEM_MONITOR_HPP_
