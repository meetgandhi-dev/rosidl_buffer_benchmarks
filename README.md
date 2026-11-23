# ROS IDL Buffer Benchmarks (`rosidl_buffer_benchmarks`)

Comprehensive evaluation, verification, and latency benchmarking of ROS 2 `rosidl` buffer backends, focusing on **Shared Memory Zero-Copy (`shmem_buffer_backend`)** compared to standard DDS wire transport.

---

## 1. Repository Structure

```text
rosidl_buffer_benchmarks/
├── shmem_bb_benchmark_ws/                 # ROS 2 colcon workspace
│   └── src/
│       ├── shmem_buffer_backend/             # Submodule: upstream shmem_buffer_backend (branch: lyrical)
│       └── shmem_buffer_backend_test/        # Test package: pub/sub nodes, launch files, test patterns
├── shmem_buffer_backend_benchmark/        # Submodule: 16-way benchmark suite (branch: develop)
│   ├── shmem_buffer_backend_benchmark/      # ROS 2 package: benchmark nodes & orchestration scripts
│   │   ├── src/shmem_buffer_backend_e2e_node.cpp
│   │   └── scripts/
│   │       ├── run_16way_benchmark.py        # Automated 16-way benchmark matrix runner
│   │       └── run_e2e_benchmark.py          # Single end-to-end benchmark run
│   ├── tools/
│   │   ├── plot_benchmark_report.py          # Figure generation from CSV results
│   │   └── plot_zenoh_comparison.py          # Zenoh vs SHM comparison plots
│   └── patches/                             # FastRTPS optimization patches (unique_ptr, lazy, reserve)
├── results/                               # Benchmark results, CSV logs, and latency plots
├── .gitmodules                            # Submodule configuration
├── .gitignore
└── README.md
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

### Option B: 1-Subscriber Selectable Mode (Shared vs. Wire)

Runs 1 publisher and 1 subscriber in either `shared` (zero-copy SHM) or `wire` (standard DDS CPU copy) mode:

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

### Option C: Automated Multi-Resolution Sweep

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

## 5. 16-Way Benchmark Suite (`shmem_buffer_backend_benchmark`)

The [`shmem_buffer_backend_benchmark`](shmem_buffer_backend_benchmark/) submodule provides a full automated benchmark matrix covering:

| Dimension | Values |
| :--- | :--- |
| **Variants** | `baseline`, `unique_ptr`, `lazy`, `reserve` |
| **Payload sizes** | 64 B, 1 KiB, 4 KiB, 16 KiB, 64 KiB, 256 KiB, 1 MiB, 4 MiB, 16 MiB |
| **Communication modes** | `inter_process`, `intra_process_va` |
| **Backends** | `cpu`, `shmem` |
| **Repeats** | 5 × 20 measured samples (10 warm-up messages excluded) |

### Building the Benchmark Package

The benchmark package lives inside `shmem_buffer_backend_benchmark/` and is built as a **separate workspace** (not inside `shmem_bb_benchmark_ws`):

```bash
source /opt/ros/lyrical/setup.bash
cd shmem_buffer_backend_benchmark
colcon build --symlink-install
source install/setup.bash
```

### Running the 16-Way Benchmark

```bash
ros2 run shmem_buffer_backend_benchmark run_16way_benchmark.py \
  --output-dir results/benchmark-results-16way \
  --overwrite
```

This runs all 720 cases (4 variants × 9 sizes × 4 paths × 5 repeats) and writes per-variant summary CSVs plus raw per-sample CSVs into the specified output directory.

### Generating Figures

Requires a Python virtual environment (from the benchmark submodule):

```bash
cd shmem_buffer_backend_benchmark
source .venv/bin/activate   # or: python -m venv .venv && pip install -r requirements.txt
python tools/plot_benchmark_report.py --data-dir ../results/benchmark-results-16way
```

### FastRTPS Optimization Patches

Three FastRTPS patches are provided under [`shmem_buffer_backend_benchmark/patches/`](shmem_buffer_backend_benchmark/patches/) to test publisher-side buffer reuse strategies:

| Patch | Description |
| :--- | :--- |
| `0001-fastrtps-reuse-unique-ptr-buffer.patch` | Reuse `unique_ptr` buffer across publishes |
| `0002-fastrtps-reuse-fastbuffer-lazy.patch` | Lazy reuse of `FastBuffer` |
| `0003-fastrtps-reuse-fastbuffer-reserve.patch` | Reserve-based `FastBuffer` reuse |

---

## 6. Benchmark Results

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
