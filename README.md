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
- The initial 10 frames (`warmup_messages:=10` by default) are published and received to allow buffer pool memory mapping, DSO registration, and DDS discovery/entity creation to stabilize.
- While warm-up frames are verified for payload integrity, they are **strictly excluded** from all latency, percentile, throughput, and jitter statistics.
- If `--max-msgs 1000` is requested with 10 warm-up frames, exactly 1010 frames are published, and exactly 1000 benchmark frames are measured.

### 3.3 Multi-Percentile Latency Analysis (Beyond Average Latency)
Relying solely on average latency masks severe latency spikes caused by queue contention, memory re-allocation, or OS scheduling. The benchmark records and reports:
- **P50 (Median)**: Represents the typical, nominal performance under sustained load.
- **P95**: Captures 95th-percentile real-time bounds.
- **P99 (Tail Latency)**: Exposes tail latency and worst-case performance crucial for real-time robotics and computer vision pipelines.
- **Mean, Min, Max, and Standard Deviation (Jitter)**: Provides full statistical characterization.
- Speedup factors are calculated across **P50 Speedup**, **P99 Speedup**, and **Mean Speedup** side-by-side.

### 3.4 Frame Tracking, Loss Detection & True Throughput
- **Sequence Numbering**: Each frame carries a monotonic sequence index (`seq = 1, 2, ...`). The subscriber detects non-contiguous sequence numbers (`seq > last_seq + 1`) to accurately count dropped frames and calculate **Frame Loss Percentage**.
- **No False FPS Assumptions**: Received frame count is **never treated as proof that target FPS was achieved**.
- **Dual Throughput Measurement**:
  - **Actual Publishing Rate (FPS)**: Calculated from publisher-side monotonic timestamps over the benchmark window.
  - **Actual Receiving Rate (FPS)**: Calculated from subscriber-side receipt timestamps over the benchmark window.

### 3.5 Outlier Transparency (3-Sigma Analysis)
- Latency outliers are **not discarded or clipped**.
- Every sample is retained in the dataset, and frames exceeding the 3-sigma threshold (`mean + 3 * std_dev`) are counted, reported, and logged with their maximum outlier value.

### 3.6 System Resource, Clock & Power Monitoring
High throughput and zero-copy performance must be evaluated in context of hardware resource utilization and power efficiency:
- **Background Non-Blocking Sampler (`SystemMonitor`)**: Runs on an independent asynchronous thread with negligible overhead (<0.05% CPU), taking periodic snapshots (200 ms interval) during the active benchmark window.
- **CPU Utilization (%)**: Reads `/proc/stat` delta tick ratios across all cores, calculating both mean utilization and peak spikes during image transport.
- **Memory & Process RSS (MB)**:
  - **Process RSS / Peak RSS (`VmRSS`, `VmHWM`)**: Directly tracks subscriber resident memory consumption from `/proc/self/status` to quantify buffer pool footprint (e.g., zero-copy memory mapping vs heap copy).
  - **System RAM Used (MB)**: Tracks host-level memory pressure from `/proc/meminfo` (`MemTotal - MemAvailable`).
- **CPU Clock Frequencies (MHz)**: Samples dynamic CPU frequency scaling (`scaling_cur_freq` / `/proc/cpuinfo`) to detect thermal throttling and governor boost behavior (mean and max MHz).
- **System Power Consumption (Watts)**: Multi-source power detection automatically reads:
  - Linux `power_supply` subsystem (`power_now` or `voltage_now * current_now` in Watts).
  - Linux RAPL energy counters (`/sys/class/powercap/intel-rapl*/energy_uj`).
  - NVIDIA Jetson Tegra INA3221 rail power sensors.
- **GPU Metrics (NVIDIA Devices)**: If an NVIDIA GPU is present, queries `nvidia-smi` for GPU core utilization (`%`), graphics clock frequency (`MHz`), and GPU power draw (`W`).

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

Runs **1 publisher** and **2 subscribers** on the same topic simultaneously:
- 1 Publisher: Allocates message in shared memory (`shmem`).
- Subscriber 1 (`subscriber_shared`): Consumes message via zero-copy shared memory (`shmem`).
- Subscriber 2 (`subscriber_fallback`): Receives fallback wire copy via DDS (`cpu`).

Automatically runs for `max_msgs` frames (with initial 10 warm-up messages excluded) and outputs side-by-side latency stats and a merged CSV:

```bash
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_two_subs_comparison.csv
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

Runs **1 CUDA publisher** and **2 subscribers** on the same topic:
- 1 Publisher: Allocates buffer in GPU memory via `cuda_buffer_backend`.
- Subscriber 1 (`cuda_subscriber`): Receives GPU memory handle via zero-copy CUDA VMM IPC (`acceptable_buffer_backends="cuda"`).
- Subscriber 2 (`fallback_subscriber`): Fallback subscriber receiving standard serialized CPU copy via DDS.

Outputs side-by-side comparison table with latency speedup:

```bash
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 \
  csv_path:=$(pwd)/results/cuda_1080p_two_subs_comparison.csv
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

### 6.1 Two-Subscriber Comparison CSV Schema (`*_fallback_two_subs.launch.py`)

| Column | Description |
| :--- | :--- |
| `resolution` | Image dimensions (e.g. `1920x1080`) |
| `size_bytes` | Payload size in bytes per frame |
| `target_hz` | Target configured publishing rate (Hz) |
| `actual_pub_hz` | Actual measured publishing rate calculated via `steady_clock` timestamps |
| `shmem_recv_hz` / `cuda_recv_hz` | Actual measured receiving rate for zero-copy subscriber (Hz) |
| `fallback_recv_hz` | Actual measured receiving rate for fallback CPU subscriber (Hz) |
| `shmem_frames` / `cuda_frames` | Total benchmark frames measured by zero-copy subscriber |
| `fallback_frames` | Total benchmark frames measured by fallback subscriber |
| `shmem_loss_pct` / `cuda_loss_pct` | Frame loss percentage for zero-copy subscriber |
| `fallback_loss_pct` | Frame loss percentage for fallback subscriber |
| `shmem_p50_ms` / `cuda_p50_ms` | **P50 (Median) latency** for zero-copy subscriber (ms) |
| `fallback_p50_ms` | **P50 (Median) latency** for fallback subscriber (ms) |
| `p50_speedup` | Factor of improvement for median latency (`fallback_p50 / zero_copy_p50`) |
| `shmem_p95_ms` / `cuda_p95_ms` | **P95 latency** for zero-copy subscriber (ms) |
| `fallback_p95_ms` | **P95 latency** for fallback subscriber (ms) |
| `shmem_p99_ms` / `cuda_p99_ms` | **P99 (Tail) latency** for zero-copy subscriber (ms) |
| `fallback_p99_ms` | **P99 (Tail) latency** for fallback subscriber (ms) |
| `p99_speedup` | Factor of improvement for tail latency (`fallback_p99 / zero_copy_p99`) |
| `shmem_mean_ms` / `cuda_mean_ms` | Mean latency for zero-copy subscriber (ms) |
| `fallback_mean_ms` | Mean latency for fallback subscriber (ms) |
| `mean_speedup` | Factor of improvement for mean latency (`fallback_mean / zero_copy_mean`) |
| `shmem_min_ms` / `fallback_min_ms` | Minimum frame latency recorded (ms) |
| `shmem_max_ms` / `fallback_max_ms` | Maximum frame latency recorded (ms) |
| `shmem_std_ms` / `fallback_std_ms` | Latency standard deviation / jitter (ms) |
| `shmem_outliers` / `fallback_outliers` | Number of frames exceeding the 3-sigma latency threshold |
| `shmem_cpu_mean_pct` / `fallback_cpu_mean_pct` | Mean host CPU utilization (%) during benchmark |
| `shmem_cpu_max_pct` / `fallback_cpu_max_pct` | Peak host CPU utilization (%) during benchmark |
| `shmem_rss_peak_mb` / `fallback_rss_peak_mb` | Peak process resident memory (`VmHWM`) in MB |
| `shmem_clock_mean_mhz` / `fallback_clock_mean_mhz` | Average CPU clock frequency (MHz) across cores |
| `shmem_power_mean_w` / `fallback_power_mean_w` | Mean system power consumption (Watts) |

### 6.2 Single-Mode and Intra-Process CSV Schema

| Column | Description |
| :--- | :--- |
| `backend` | Transport backend tested (`shmem`, `cuda`, or `cpu`) |
| `size_bytes` | Payload size in bytes per frame |
| `resolution` | Image dimensions (e.g. `1920x1080`) |
| `target_hz` | Target configured rate (Hz) |
| `actual_pub_hz` | Actual publisher FPS measured via monotonic timestamps |
| `actual_recv_hz` | Actual subscriber receiving FPS |
| `total_sent` | Total sequence range sent in benchmark window |
| `received_frames` | Frames successfully received and measured |
| `lost_frames` | Dropped/skipped frames identified via sequence numbering |
| `loss_pct` | Frame loss percentage (`lost_frames / total_sent * 100`) |
| `mean_latency_ms` | Average frame latency (ms) |
| `min_latency_ms` | Minimum frame latency (ms) |
| `p50_latency_ms` | 50th percentile (Median) latency (ms) |
| `p95_latency_ms` | 95th percentile latency (ms) |
| `p99_latency_ms` | 99th percentile (Tail) latency (ms) |
| `max_latency_ms` | Maximum frame latency (ms) |
| `std_dev_ms` | Latency standard deviation (ms) |
| `outliers_3sigma_count` | Frames exceeding `mean + 3 * std_dev` |
| `cpu_util_mean_pct` | Mean host CPU utilization (%) |
| `cpu_util_max_pct` | Peak host CPU utilization (%) |
| `process_rss_mean_mb` | Average process RSS resident memory (MB) |
| `process_rss_peak_mb` | Peak process RSS resident memory (`VmHWM`) in MB |
| `system_ram_mb` | Host system RAM used (MB) |
| `cpu_clock_mean_mhz` | Mean CPU core clock frequency (MHz) |
| `cpu_clock_max_mhz` | Peak CPU boost clock frequency (MHz) |
| `power_mean_w` | Mean system power draw (Watts) |
| `power_max_w` | Peak system power draw (Watts) |
| `gpu_util_pct` | Mean NVIDIA GPU utilization (%) |
| `gpu_clock_mhz` | Mean NVIDIA GPU core clock (MHz) |
| `gpu_power_w` | Mean NVIDIA GPU power draw (Watts) |

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
