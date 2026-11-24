# ROS IDL Buffer Benchmarks (`rosidl_buffer_benchmarks`)

Comprehensive evaluation, verification, and latency benchmarking of ROS 2 `rosidl` buffer backends, comparing **Zero-Copy Transports (`shmem_buffer_backend`, `cuda_buffer_backend`)** against standard DDS wire transport.

---

## 1. Repository Structure

```text
rosidl_buffer_benchmarks/
├── shmem_bb_benchmark_ws/          # Shared Memory Benchmark Workspace
│   └── src/
│       ├── shmem_buffer_backend/      # Git submodule: upstream shmem_buffer_backend (branch: lyrical)
│       └── shmem_bb_benchmark/        # Benchmark package: nodes, launch scripts, test patterns
├── cuda_bb_benchmark_ws/           # CUDA Buffer Benchmark Workspace
│   └── src/
│       ├── rosidl_buffer_backends/    # Git submodule: upstream rosidl_buffer_backends (tag: 0.1.2)
│       └── cuda_bb_benchmark/         # Benchmark package: CUDA pub/sub nodes, launch scripts, sweep
├── results/                        # Benchmark results, CSV logs, and latency plots
├── .gitmodules                     # Submodule configurations
├── .gitignore
└── README.md                       # Project overview, usage instructions, and results
```

---

## 2. Prerequisites

1. **Ubuntu 26.04** with ROS 2 **Lyrical** (preferred)
2. **Ubuntu 24.04** with ROS 2 **Lyrical** (pre-built Debian packages available via [`ros2_tier3_packages`](https://github.com/meetgandhi-dev/ros2_tier3_packages))
3. **Architecture**: `amd64` (x86_64) or `arm64` (aarch64)
4. **For CUDA Benchmarks**:
   - NVIDIA GPU with NVIDIA Driver installed
   - CUDA Toolkit >= 11.8

---

## 3. Benchmark Methodology

To ensure statistically rigorous, repeatable, and transparent performance comparisons, the benchmarking suite adheres to the following methodology:

### 3.1 Consistent Monotonic Timing (`std::chrono::steady_clock`)
- Both publisher and subscriber sample `std::chrono::steady_clock` (which maps to `CLOCK_MONOTONIC` on Linux).
- Monotonic timestamps are encoded directly in `msg.header.stamp` (`sec` and `nanosec`) and embedded inside `msg.header.frame_id`.
- This eliminates clock drift, NTP adjustments, and leap second artifacts across separate processes on the same host, guaranteeing true nanosecond-accurate latency measurements.

### 3.2 Warm-Up Frame Exclusion
- The initial frames (`warmup_messages:=10` by default) are published and received to allow buffer pool memory mapping, DSO registration, and DDS discovery/entity creation to stabilize.
- While warm-up frames are verified for payload integrity, they are **strictly excluded** from all latency, percentile, throughput, and jitter statistics.
- If `--max-msgs 1000` is requested with 10 warm-up frames, exactly 1010 frames are published, and exactly 1000 benchmark frames are measured.

### 3.3 Latency and Throughput Measurements

For every benchmark execution, the following metrics are recorded:
- **Actual Publishing FPS**: Calculated from publisher monotonic timestamps over the active benchmark window.
- **Actual Receiving FPS**: Calculated from subscriber monotonic receipt timestamps over the active benchmark window.
- **Total Published Frames**: Total messages dispatched during the benchmark window.
- **Total Received Frames**: Messages successfully received and processed.
- **Dropped Frames**: Missing or skipped frames detected via strictly incrementing sequence numbers.
- **Frame-Loss Percentage**: `(dropped_frames / total_published_frames) * 100%`.
- **Latency Distribution**:
  - **P50 (Median)**: Nominal sustained performance under load.
  - **P95**: Real-time upper bound.
  - **P99 (Tail Latency)**: Worst-case latency critical for robotic safety.
  - **Mean, Minimum, Maximum, and Standard Deviation (Jitter)**.
- **3-Sigma Outliers**: Number of frames exceeding `mean + 3 * std_dev`. **Outliers are always reported and never discarded**.
- **No False FPS Assumptions**: Received frame count is **never treated as proof that target FPS was achieved**.

### 3.4 CUDA Buffer Backend Latency Separation

For CUDA benchmarks (`cuda_bb_benchmark`), latency is decomposed to reflect GPU pipeline realities:
1. **Subscriber Callback Entry Timestamp**: Monotonic timestamp recorded at the exact moment the ROS 2 subscriber callback is entered (`t_cb_entry`).
2. **Buffer-Ready Timestamp**: Monotonic timestamp recorded immediately after the required CUDA synchronization (`cudaStreamSynchronize`) completes and the buffer is ready for GPU compute (`t_buf_ready`).
3. **Publisher-to-Buffer-Ready Latency**: End-to-end latency from publish timestamp to GPU buffer readiness.
4. **Strict Latency Separation**: Callback latency (`t_cb_entry - t_pub`) and buffer-ready latency (`t_buf_ready - t_pub`) are **kept separate** in both console summaries and CSV outputs.
5. **CUDA Sync Overhead**: Direct measurement of stream synchronization time (`t_buf_ready - t_cb_entry`).

### 3.5 System Resource Monitoring

Resource telemetry runs on an asynchronous non-blocking thread (`SystemMonitor`) sampling process and system metrics every 200 ms:
- **Publisher CPU Utilization**: Dedicated process-level CPU % (`utime + stime` ticks delta divided by system ticks, scaled across available CPU cores; reports mean and peak).
- **Subscriber CPU Utilization**: Dedicated process-level CPU % for the subscriber process (mean and peak).
- **Publisher Process RSS**: Peak resident memory (`VmHWM` from `/proc/self/status`) consumed by the publisher process.
- **Subscriber Process RSS**: Peak resident memory (`VmHWM` from `/proc/self/status`) consumed by the subscriber process (reflecting buffer pool mapping vs heap duplication).
- **System RAM Usage**: Overall host memory pressure (`MemTotal - MemAvailable` from `/proc/meminfo`).
- **GPU Utilization**: Where reliable monitoring is available (`nvidia-smi`), records GPU core utilization (%); otherwise reports `N/A`.
- **Strict Telemetry Integrity**: Unsupported or unavailable metrics are recorded as `N/A`—unreliable or synthetic telemetry is never injected just to populate CSV columns.

---

## 4. Getting Started

### Clone with Submodules

To clone this repository and initialize all submodules in one step:

```bash
git clone --recursive <REPO_URL> rosidl_buffer_benchmarks
cd rosidl_buffer_benchmarks
```

If already cloned without `--recursive`, fetch and initialize submodules:

```bash
git submodule update --init --recursive
```

### Workspace Helper Scripts (`install_deps.sh` & `build.sh`)

Each workspace includes dedicated helper scripts to install dependencies via `rosdep` and build only the required packages using `--packages-up-to`:

#### 1. Shared Memory Workspace (`shmem_bb_benchmark_ws`)

```bash
cd shmem_bb_benchmark_ws

# Install dependencies (runs rosdep for shmem_bb_benchmark and its deps)
./install_deps.sh

# Build workspace (uses colcon build --packages-up-to shmem_bb_benchmark)
./build.sh

# Source environment
source install/setup.bash
cd ..
```

#### 2. CUDA Workspace (`cuda_bb_benchmark_ws`)
*(Requires system with NVIDIA GPU and CUDA Toolkit)*

```bash
cd cuda_bb_benchmark_ws

# Install dependencies via rosdep (targets cuda_bb_benchmark without unneeded extras)
./install_deps.sh

# Build workspace (uses colcon build --packages-up-to cuda_bb_benchmark)
./build.sh

# Source environment
source install/setup.bash
cd ..
```

---

## 5. Running Benchmarks

### 4.1. Shared Memory (`shmem`) Benchmarks

Always source the built workspace before launching nodes:

```bash
source shmem_bb_benchmark_ws/install/setup.bash
```

#### Option A: 2-Subscriber Comparison (Shared vs. Fallback Side-by-Side)

Runs **1 publisher** (always in SHMEM mode) and allows controlling which subscribers run via `sub_mode`:
- `sub_mode:=both` (default): Runs both zero-copy SHMEM subscriber and CPU fallback subscriber simultaneously on the same topic.
- `sub_mode:=shmem`: Runs only the zero-copy SHMEM subscriber (`subscriber_shared`).
- `sub_mode:=cpu`: Runs only the CPU fallback subscriber (`subscriber_fallback`).

Automatically runs for `max_msgs` frames (with initial 10 warm-up messages excluded) and outputs side-by-side latency stats and a merged CSV:

```bash
# Run both subscribers (default side-by-side comparison):
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=both \
  csv_path:=$(pwd)/results/1080p_30fps_two_subs_comparison.csv

# Run 1 SHMEM pub with only 1 SHMEM sub:
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  sub_mode:=shmem max_msgs:=1000

# Run 1 SHMEM pub with only 1 CPU sub:
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  sub_mode:=cpu max_msgs:=1000
```

#### Option B: 1-Subscriber Selectable Mode (Inter-Process: Shared vs. Wire)

Runs 1 publisher and 1 subscriber **in separate processes** in either `shared` (zero-copy SHM) or `wire` (standard DDS CPU copy) mode:

```bash
# 1. Run in Zero-Copy Shared Memory Mode:
ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shared max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_shmem.csv

# 2. Run in Wire (CPU fallback) Mode:
ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_wire.csv
```

#### Option C: Intra-Process Comparison (Same Process: SHM vs. CPU)

Runs both publisher and subscriber **inside the same OS process** using a `ComposableNodeContainer`
with `use_intra_process_comms: True`. The message `shared_ptr` is delivered directly with no
serialization, no DDS overhead, and no inter-process copy at all:

```bash
# Intra-process with shared memory backend:
ros2 launch shmem_bb_benchmark intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shmem max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_intra_shmem.csv

# Intra-process with CPU backend:
ros2 launch shmem_bb_benchmark intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cpu max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_intra_cpu.csv
```

#### Shared Memory Transport Coverage Summary

| Path | Communication | Backend | Launch File |
|:---|:---:|:---:|:---|
| SHM zero-copy + wire fallback | Inter-process | `shmem` + `cpu` | `shmem_fallback_two_subs.launch.py` |
| SHM zero-copy only | Inter-process | `shmem` | `shmem_or_wire_pub_sub.launch.py mode:=shared` |
| DDS wire/serialized only | Inter-process | `cpu` | `shmem_or_wire_pub_sub.launch.py mode:=wire` |
| SHM backend, same process | Intra-process | `shmem` | `intra_process_pub_sub.launch.py mode:=shmem` |
| CPU backend, same process | Intra-process | `cpu` | `intra_process_pub_sub.launch.py mode:=cpu` |

#### Option D: Automated Multi-Resolution Sweep

```bash
ros2 run shmem_bb_benchmark benchmark_latency.py \
  --sizes 640x480 1280x720 1920x1080 3840x2160 \
  --rates 10 30 60 \
  --max-frames 1000 \
  --backends both \
  --output results/shmem_latency_sweep_summary.csv
```

---

### 4.2. CUDA Buffer Backend (`cuda_bb_benchmark`) Benchmarks

Source the CUDA workspace before launching:

```bash
source cuda_bb_benchmark_ws/install/setup.bash
```

#### Option A: 2-Subscriber Comparison (CUDA Zero-Copy vs. Fallback Side-by-Side)

Runs **1 CUDA publisher** (always allocating GPU memory) and allows controlling which subscribers run via `sub_mode`:
- `sub_mode:=both` (default): Runs both CUDA zero-copy subscriber and CPU fallback subscriber simultaneously on the same topic.
- `sub_mode:=cuda`: Runs only the CUDA zero-copy subscriber (`cuda_subscriber`).
- `sub_mode:=cpu`: Runs only the CPU fallback subscriber (`fallback_subscriber`).

Outputs side-by-side comparison table with latency speedup:

```bash
# Run both subscribers (default side-by-side comparison):
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=both \
  csv_path:=$(pwd)/results/cuda_1080p_two_subs_comparison.csv

# Run 1 CUDA pub with only 1 CUDA sub:
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  sub_mode:=cuda max_msgs:=1000

# Run 1 CUDA pub with only 1 CPU sub:
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  sub_mode:=cpu max_msgs:=1000
```

#### Option B: 1-Subscriber Selectable Mode (Inter-Process: CUDA vs. Wire)

Runs 1 publisher and 1 subscriber in separate processes:

```bash
# 1. Run in Zero-Copy CUDA IPC Mode:
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000 \
  csv_path:=$(pwd)/results/cuda_1080p_ipc.csv

# 2. Run in Wire (CPU copy) Mode:
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000 \
  csv_path:=$(pwd)/results/cuda_1080p_wire.csv
```

#### Option C: Intra-Process Comparison (Same Process: CUDA vs. CPU)

Runs publisher and subscriber in the same process using `ComposableNodeContainer` with `use_intra_process_comms=True`:

```bash
# Intra-process with CUDA buffer backend:
ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000 \
  csv_path:=$(pwd)/results/cuda_1080p_intra_cuda.csv

# Intra-process with CPU buffer:
ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cpu max_msgs:=1000 \
  csv_path:=$(pwd)/results/cuda_1080p_intra_cpu.csv
```

#### CUDA Transport Coverage Summary

| Path | Communication | Backend | Launch File |
|:---|:---:|:---:|:---|
| CUDA zero-copy + wire fallback | Inter-process | `cuda` + `cpu` | `cuda_fallback_two_subs.launch.py` |
| CUDA zero-copy only | Inter-process | `cuda` | `cuda_or_wire_pub_sub.launch.py mode:=cuda` |
| DDS wire/serialized only | Inter-process | `cpu` | `cuda_or_wire_pub_sub.launch.py mode:=wire` |
| CUDA backend, same process | Intra-process | `cuda` | `cuda_intra_process_pub_sub.launch.py mode:=cuda` |
| CPU backend, same process | Intra-process | `cpu` | `cuda_intra_process_pub_sub.launch.py mode:=cpu` |

#### Option D: Automated Multi-Resolution Sweep

```bash
ros2 run cuda_bb_benchmark benchmark_latency.py \
  --sizes 640x480 1280x720 1920x1080 3840x2160 \
  --rates 10 30 60 \
  --max-frames 1000 \
  --backends both \
  --output results/cuda_latency_sweep_summary.csv
```

---

## 6. Benchmark Results & Output Schemas

All benchmark CSVs and logs are saved under [`results/`](results/).

### 6.1 Two-Subscriber Comparison CSV Schema

#### Shared Memory Comparison (`shmem_fallback_two_subs.launch.py`)

| Column | Description |
| :--- | :--- |
| `resolution` | Image dimensions (e.g. `1920x1080`) |
| `size_bytes` | Payload size in bytes per frame |
| `target_hz` | Target configured publishing rate (Hz) |
| `actual_pub_fps` | Actual publisher FPS measured via monotonic `steady_clock` timestamps |
| `shmem_recv_fps` | Actual receiving FPS for zero-copy SHMEM subscriber |
| `fallback_recv_fps` | Actual receiving FPS for fallback CPU subscriber |
| `total_published_frames` | Total messages dispatched during the benchmark window |
| `shmem_received_frames` | Benchmark frames received by SHMEM subscriber |
| `fallback_received_frames` | Benchmark frames received by fallback subscriber |
| `shmem_dropped_frames` | Dropped frames detected for SHMEM subscriber |
| `fallback_dropped_frames` | Dropped frames detected for fallback subscriber |
| `shmem_loss_pct` | Frame loss percentage for SHMEM subscriber |
| `fallback_loss_pct` | Frame loss percentage for fallback subscriber |
| `shmem_p50_ms` / `fallback_p50_ms` | **P50 (Median) latency** (ms) |
| `p50_speedup` | Factor of improvement for median latency (`fallback_p50 / shmem_p50`) |
| `shmem_p95_ms` / `fallback_p95_ms` | **P95 latency** (ms) |
| `shmem_p99_ms` / `fallback_p99_ms` | **P99 (Tail) latency** (ms) |
| `p99_speedup` | Factor of improvement for tail latency (`fallback_p99 / shmem_p99`) |
| `shmem_mean_ms` / `fallback_mean_ms` | Mean latency (ms) |
| `mean_speedup` | Factor of improvement for mean latency (`fallback_mean / shmem_mean`) |
| `shmem_min_ms` / `fallback_min_ms` | Minimum frame latency recorded (ms) |
| `shmem_max_ms` / `fallback_max_ms` | Maximum frame latency recorded (ms) |
| `shmem_std_ms` / `fallback_std_ms` | Latency standard deviation / jitter (ms) |
| `shmem_outliers` / `fallback_outliers` | Number of 3-sigma outliers (`> mean + 3 * std_dev`) (never discarded) |
| `pub_cpu_mean_pct` / `pub_cpu_max_pct` | Publisher process CPU utilization (%) (mean / peak) |
| `shmem_sub_cpu_mean_pct` / `shmem_sub_cpu_max_pct` | SHMEM subscriber process CPU utilization (%) (mean / peak) |
| `fallback_sub_cpu_mean_pct` / `fallback_sub_cpu_max_pct` | Fallback subscriber process CPU utilization (%) (mean / peak) |
| `pub_rss_peak_mb` | Publisher process peak resident memory (`VmHWM`) in MB |
| `shmem_sub_rss_peak_mb` | SHMEM subscriber process peak resident memory (`VmHWM`) in MB |
| `fallback_sub_rss_peak_mb` | Fallback subscriber process peak resident memory (`VmHWM`) in MB |
| `system_ram_mb` | Host system RAM used (MB) |
| `gpu_util_pct` | GPU core utilization (%) (from Jetson `tegrastats` `GR3D_FREQ`, sysfs, or `nvidia-smi`) |
| `gpu_freq_mhz` | GPU core frequency in MHz (from Jetson `tegrastats` `GR3D_FREQ` or devfreq) |
| `emc_util_pct` | EMC memory controller utilization (%) (from Jetson `tegrastats` `EMC_FREQ`) |
| `emc_freq_mhz` | EMC memory controller bus clock in MHz (from Jetson `tegrastats` `EMC_FREQ` or devfreq) |
| `cpu_temp_c` | CPU temperature in °C (from Jetson `tegrastats` or thermal zones) |
| `gpu_temp_c` | GPU temperature in °C (from Jetson `tegrastats` or thermal zones) |

#### CUDA Comparison (`cuda_fallback_two_subs.launch.py`)

| Column | Description |
| :--- | :--- |
| `resolution` | Image dimensions (e.g. `1920x1080`) |
| `size_bytes` | Payload size in bytes per frame |
| `target_hz` | Target configured publishing rate (Hz) |
| `actual_pub_fps` | Actual publisher FPS measured via monotonic timestamps |
| `cuda_recv_fps` / `fallback_recv_fps` | Actual receiving FPS for CUDA and fallback subscribers |
| `total_published_frames` | Total messages dispatched during the benchmark window |
| `cuda_received_frames` / `fallback_received_frames` | Frames received and measured |
| `cuda_dropped_frames` / `fallback_dropped_frames` | Dropped frames detected |
| `cuda_loss_pct` / `fallback_loss_pct` | Frame loss percentage |
| `cuda_cb_*` | **Callback Entry Latency**: P50, P95, P99, Mean, Min, Max, StdDev, Outliers (`t_cb_entry - t_pub`) |
| `cuda_br_*` | **Buffer-Ready Latency**: P50, P95, P99, Mean, Min, Max, StdDev, Outliers after `cudaStreamSynchronize` |
| `fallback_*` | Fallback CPU subscriber latencies |
| `p50_speedup` / `p99_speedup` / `mean_speedup` | Speedup ratios comparing Fallback latency to CUDA Buffer-Ready latency |
| `pub_cpu_mean_pct` / `pub_cpu_max_pct` | Publisher process CPU utilization (%) (mean / peak) |
| `cuda_sub_cpu_mean_pct` / `cuda_sub_cpu_max_pct` | CUDA subscriber process CPU utilization (%) (mean / peak) |
| `fallback_sub_cpu_mean_pct` / `fallback_sub_cpu_max_pct` | Fallback subscriber process CPU utilization (%) (mean / peak) |
| `pub_rss_peak_mb` / `cuda_sub_rss_peak_mb` / `fallback_sub_rss_peak_mb` | Peak process resident memory (`VmHWM`) in MB |
| `system_ram_mb` | Host system RAM used (MB) |
| `gpu_util_pct` | GPU core utilization (%) (from Jetson `tegrastats` `GR3D_FREQ`, sysfs, or `nvidia-smi`) |
| `gpu_freq_mhz` | GPU core frequency in MHz (from Jetson `tegrastats` `GR3D_FREQ` or devfreq) |
| `emc_util_pct` | EMC memory controller utilization (%) (from Jetson `tegrastats` `EMC_FREQ`) |
| `emc_freq_mhz` | EMC memory controller bus clock in MHz (from Jetson `tegrastats` `EMC_FREQ` or devfreq) |
| `cpu_temp_c` | CPU temperature in °C (from Jetson `tegrastats` or thermal zones) |
| `gpu_temp_c` | GPU temperature in °C (from Jetson `tegrastats` or thermal zones) |

### 6.2 Single-Mode and Intra-Process CSV Schema

| Column | Description |
| :--- | :--- |
| `backend` | Transport backend tested (`shmem`, `cuda`, or `cpu`) |
| `size_bytes` | Payload size in bytes per frame |
| `resolution` | Image dimensions (e.g. `1920x1080`) |
| `target_hz` | Target configured rate (Hz) |
| `actual_pub_fps` | Actual publisher FPS measured via monotonic timestamps |
| `actual_recv_fps` | Actual subscriber receiving FPS |
| `total_published_frames` | Total sequence range sent in benchmark window |
| `total_received_frames` | Frames successfully received and measured |
| `dropped_frames` | Dropped/skipped frames identified via sequence numbering |
| `frame_loss_pct` | Frame loss percentage (`dropped_frames / total_published_frames * 100`) |
| `mean_latency_ms` / `cb_mean_latency_ms` | Mean frame latency (ms) |
| `min_latency_ms` / `cb_min_latency_ms` | Minimum frame latency (ms) |
| `p50_latency_ms` / `cb_p50_latency_ms` | 50th percentile (Median) latency (ms) |
| `p95_latency_ms` / `cb_p95_latency_ms` | 95th percentile latency (ms) |
| `p99_latency_ms` / `cb_p99_latency_ms` | 99th percentile (Tail) latency (ms) |
| `max_latency_ms` / `cb_max_latency_ms` | Maximum frame latency (ms) |
| `std_dev_ms` / `cb_std_dev_ms` | Latency standard deviation / jitter (ms) |
| `outliers_3sigma_count` / `cb_outliers_3sigma_count` | Frames exceeding `mean + 3 * std_dev` (never discarded) |
| `br_*` *(CUDA only)* | Buffer-ready latency statistics after `cudaStreamSynchronize` (Mean, Min, P50, P95, P99, Max, StdDev, Outliers) |
| `pub_cpu_mean_pct` / `pub_cpu_max_pct` | Publisher process CPU utilization (%) (or container in intra-process) |
| `sub_cpu_mean_pct` / `sub_cpu_max_pct` | Subscriber process CPU utilization (%) (or container in intra-process) |
| `pub_rss_peak_mb` | Publisher process peak resident memory (`VmHWM`) in MB |
| `sub_rss_peak_mb` | Subscriber process peak resident memory (`VmHWM`) in MB |
| `system_ram_mb` | Host system RAM used (MB) |
| `gpu_util_pct` | GPU core utilization (%) (from Jetson `tegrastats` `GR3D_FREQ`, sysfs, or `nvidia-smi`) |
| `gpu_freq_mhz` | GPU core frequency in MHz (from Jetson `tegrastats` `GR3D_FREQ` or devfreq) |
| `emc_util_pct` | EMC memory controller utilization (%) (from Jetson `tegrastats` `EMC_FREQ`) |
| `emc_freq_mhz` | EMC memory controller bus clock in MHz (from Jetson `tegrastats` `EMC_FREQ` or devfreq) |
| `cpu_temp_c` | CPU temperature in °C (from Jetson `tegrastats` or thermal zones) |
| `gpu_temp_c` | GPU temperature in °C (from Jetson `tegrastats` or thermal zones) |

---

## 7. Reproduction Steps

To quickly reproduce benchmark runs:

```bash
# 1. Clone repository recursively
git clone --recursive <REPO_URL> rosidl_buffer_benchmarks
cd rosidl_buffer_benchmarks

# 2. Build Shared Memory Workspace
cd shmem_bb_benchmark_ws
./install_deps.sh
./build.sh
source install/setup.bash
cd ..

# 3. Reproduce 1080p @ 30 FPS SHM vs Fallback Benchmark (1000 frames)
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_two_subs_comparison.csv

# 4. Reproduce Intra-Process Zero-Copy vs CPU Benchmark (1000 frames)
ros2 launch shmem_bb_benchmark intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shmem max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_intra_shmem.csv
```
