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

### Build the Workspaces

#### Building Shared Memory Workspace

```bash
cd shmem_bb_benchmark_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
cd ..
```

#### Building CUDA Workspace (on systems with NVIDIA GPU & CUDA Toolkit)

```bash
cd cuda_bb_benchmark_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
```

---

## 4. Running Benchmarks

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

## 5. Benchmark Results

All benchmark CSVs and logs are saved under [`results/`](results/).

### Merged CSV Metric Schema

| Column | Description |
| :--- | :--- |
| `resolution` | Image dimensions (e.g., `1920x1080`) |
| `size_bytes` | Payload size in bytes per frame |
| `target_hz` | Target publishing rate |
| `frame_cap` | Total frames measured (`max_msgs`, excluding initial warm-up frames) |
| `mean_ms` / `avg_latency_ms` | Average latency for subscriber (ms) |
| `min_latency_ms` / `max_latency_ms` | Minimum and maximum recorded frame latency (ms) |
| `std_dev_ms` | Latency standard deviation (jitter) |
| `speedup` | Factor of improvement (`fallback / zero_copy`) |
