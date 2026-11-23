# ROS IDL Buffer Benchmarks (`rosidl_buffer_benchmarks`)

Comprehensive evaluation, verification, and latency benchmarking of ROS 2 `rosidl` buffer backends, focusing on **Shared Memory Zero-Copy (`shmem_buffer_backend`)** compared to standard DDS wire transport.

---

## 1. Repository Structure

```text
rosidl_buffer_benchmarks/
├── shmem_bb_benchmark_ws/          # ROS 2 colcon workspace
│   └── src/
│       ├── shmem_buffer_backend/      # Git submodule: upstream shmem_buffer_backend (branch: lyrical)
│       └── shmem_buffer_backend_test/ # Benchmark package: nodes, launch scripts, test patterns
├── results/                        # Benchmark results, CSV logs, and latency plots
├── .gitmodules                     # Submodule configuration
├── .gitignore
└── README.md                       # Project overview, usage instructions, and results
```

---

## 2. Prerequisites

- **Operating System**: Ubuntu 24.04 LTS (Noble) or Ubuntu 22.04 LTS (Jammy)
- **Target Architectures**: `x86_64` (AMD64) or `aarch64` (ARM64 / NVIDIA Jetson AGX Orin)
- **ROS 2 Distribution**: ROS 2 **Lyrical** (custom-patched ROS 2 providing `acceptable_buffer_backends`).
  - Pre-built Debian packages for AMD64 and ARM64 are available via [`ros2_tier3_packages`](https://github.com/meetgandhi-dev/ros2_tier3_packages).

---

## 3. Getting Started

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

### Build the Workspace

```bash
# Navigate to the workspace
cd shmem_bb_benchmark_ws

# Source ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash

# Build packages
colcon build --symlink-install
```

---

## 4. Running Benchmarks

Always source the built workspace before launching nodes:

```bash
source shmem_bb_benchmark_ws/install/setup.bash
```

### Option A: 2-Subscriber Comparison (Shared vs. Fallback Side-by-Side)

Runs **1 publisher** and **2 subscribers** on the same topic simultaneously:
- 1 Publisher: Allocates message in shared memory (`shmem`).
- Subscriber 1 (`subscriber_shared`): Consumes message via zero-copy shared memory (`shmem`).
- Subscriber 2 (`subscriber_fallback`): Receives fallback wire copy via DDS (`cpu`).

Automatically runs for `max_msgs` frames and outputs side-by-side latency stats and a merged CSV:

```bash
ros2 launch shmem_buffer_backend_test shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_two_subs_comparison.csv
```

### Option B: 1-Subscriber Selectable Mode (Inter-Process: Shared vs. Wire)

Runs 1 publisher and 1 subscriber **in separate processes** in either `shared` (zero-copy SHM) or `wire` (standard DDS CPU copy) mode:

```bash
# 1. Run in Zero-Copy Shared Memory Mode:
ros2 launch shmem_buffer_backend_test shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shared max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_shmem.csv

# 2. Run in Wire (CPU fallback) Mode:
ros2 launch shmem_buffer_backend_test shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_wire.csv
```

### Option C: Intra-Process Comparison (Same Process: SHM vs. CPU)

Runs both publisher and subscriber **inside the same OS process** using a `ComposableNodeContainer`
with `use_intra_process_comms: True`. The message `shared_ptr` is delivered directly with no
serialization, no DDS overhead, and no inter-process copy at all. Compares SHM vs CPU backends
purely at the intra-process level:

```bash
# Intra-process with shared memory backend:
ros2 launch shmem_buffer_backend_test intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shmem max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_intra_shmem.csv

# Intra-process with CPU backend:
ros2 launch shmem_buffer_backend_test intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cpu max_msgs:=1000 \
  csv_path:=$(pwd)/results/1080p_30fps_intra_cpu.csv
```

### Transport Coverage Summary

| Path | Communication | Backend | Launch File |
|:---|:---:|:---:|:---|
| SHM zero-copy + wire fallback | Inter-process | `shmem` + `cpu` | `shmem_fallback_two_subs.launch.py` |
| SHM zero-copy only | Inter-process | `shmem` | `shmem_or_wire_pub_sub.launch.py mode:=shared` |
| DDS wire/serialized only | Inter-process | `cpu` | `shmem_or_wire_pub_sub.launch.py mode:=wire` |
| SHM backend, same process | Intra-process | `shmem` | `intra_process_pub_sub.launch.py mode:=shmem` |
| CPU backend, same process | Intra-process | `cpu` | `intra_process_pub_sub.launch.py mode:=cpu` |

### Option D: Automated Multi-Resolution Sweep

Run multi-resolution/multi-rate sweeps across backends using the benchmarking script:

```bash
ros2 run shmem_buffer_backend_test benchmark_latency.py \
  --sizes 640x480 1280x720 1920x1080 3840x2160 \
  --rates 10 30 60 \
  --max-frames 1000 \
  --backends both \
  --output results/latency_sweep_summary.csv
```

---

## 5. Benchmark Results

All benchmark CSVs and logs are saved under [`results/`](results/).

### Merged CSV Metric Schema

| Column | Description |
| :--- | :--- |
| `resolution` | Image dimensions (e.g., `1920x1080`) |
| `size_bytes` | Payload size in bytes per frame |
| `target_hz` | Target publishing rate |
| `frame_cap` | Total frames measured (`max_msgs`) |
| `shmem_avg_latency_ms` | Average latency for shared memory zero-copy subscriber (ms) |
| `fallback_avg_latency_ms` | Average latency for fallback DDS subscriber (ms) |
| `latency_diff_ms` | `fallback_avg_latency_ms - shmem_avg_latency_ms` |
| `speedup` | Factor of improvement (`fallback / shmem`) |
| `shmem_std_dev_ms` / `fallback_std_dev_ms` | Latency standard deviation (jitter) |

### Summary Comparison Table

*Sample results across standard resolutions (1000 frames @ 30 Hz):*

| Resolution | Payload Size | Rate (Hz) | SHMEM Latency (ms) | Fallback/Wire Latency (ms) | Latency Reduction | Speedup |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **640x480** (VGA) | 0.30 MB | 30 | *TBD* | *TBD* | *TBD* | *TBD* |
| **1280x720** (HD) | 0.92 MB | 30 | *TBD* | *TBD* | *TBD* | *TBD* |
| **1920x1080** (FHD) | 2.07 MB | 30 | *TBD* | *TBD* | *TBD* | *TBD* |
| **3840x2160** (4K) | 8.29 MB | 30 | *TBD* | *TBD* | *TBD* | *TBD* |
