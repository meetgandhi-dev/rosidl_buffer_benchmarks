# cuda_bb_benchmark

Benchmarking and evaluation package for **`cuda_buffer_backend`** in ROS 2.

This package benchmarks zero-copy GPU memory sharing via CUDA IPC (Virtual Memory Management) versus standard DDS wire serialization and CPU copies, as well as intra-process communication.

---

## 1. Overview & Capabilities

- **Zero-Copy CUDA IPC**: Uses `cuda_buffer_backend` and NVIDIA CUDA VMM IPC to transport GPU memory handles directly across ROS 2 nodes on the same host without PCIe / host staging copies.
- **Fair GPU-Ready End-to-End Latency Comparison**:
  - **CUDA-Native Path**: Publisher allocates CUDA buffer on GPU -> transports via CUDA IPC -> Subscriber imports handle via `cuda_buffer_backend::from_input_buffer()` and synchronizes stream.
  - **CPU-Backed Path**: Publisher allocates CPU buffer in host RAM -> transports via standard DDS -> Subscriber receives on host -> copies payload to pre-allocated destination GPU buffer via `cudaMemcpyAsync(H2D)` and synchronizes stream.
  - **Identical Endpoint**: Both paths are measured until the full payload is ready and synchronized in subscriber GPU memory.
  - **Isolated Breakdown**: Callback entry latency, device operation overhead (H2D copy vs CUDA import/sync), and GPU-ready latency are recorded and reported separately.
- **Inter-Process Benchmarks**:
  - `cuda_or_wire_pub_sub.launch.py`: Compares `mode:=cuda` (zero-copy) vs `mode:=wire` (standard DDS CPU copy) across configurable resolutions, rates, and message caps.
  - `cuda_fallback_two_subs.launch.py`: Runs 1 CUDA publisher with 2 concurrent subscribers (one zero-copy GPU subscriber, one standard CPU fallback subscriber with H2D transfer) to measure transport and GPU ingestion overhead side-by-side.
- **Intra-Process Benchmarks**:
  - `cuda_intra_process_pub_sub.launch.py`: Pub and Sub loaded into the same `component_container` with intra-process communication enabled.
- **Warm-Up Exclusion**: Automatically ignores initial warm-up messages (`warmup_messages:=10` by default) to eliminate setup, allocator initialization, and DDS discovery transients from the statistics.
- **Iteration Benchmark Script**:
  - `run_iterations.py`: Runs repeated benchmark iterations for a single resolution (min: 1, max: 10) and calculates individual and averaged results.

---

## 2. Requirements

- Ubuntu 26.04 or 24.04 (`amd64` or `arm64`)
- ROS 2 Lyrical (rolling development branch)
- NVIDIA GPU with CUDA Toolkit >= 11.8 and NVIDIA Display Drivers
- `cuda_buffer_backend` dependencies (`cuda_buffer`, `cuda_buffer_backend_msgs`, `rosidl_buffer`)

---

## 3. Build Instructions

```bash
cd ~/test/rosidl_buffer_benchmarks/cuda_bb_benchmark_ws
source /opt/ros/lyrical/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. Running Benchmarks

### 4.1 Single Mode (CUDA vs Wire)

```bash
# CUDA Zero-Copy (1080p @ 30 Hz, 1000 benchmark frames)
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000

# Wire Mode (Standard DDS CPU serialization)
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000
```

### 4.2 Side-by-Side Zero-Copy vs Fallback

```bash
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000
```

### 4.3 Intra-Process (Same Process)

```bash
# Intra-process with CUDA backend
ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000

# Intra-process with CPU backend
ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cpu max_msgs:=1000
```

### 4.4 Repeated Iterations & Averages

```bash
ros2 run cuda_bb_benchmark run_iterations.py \
  --iterations 5 \
  --resolution 1920x1080 \
  --rate 30.0 \
  --max-msgs 1000 \
  --sub-mode both \
  --output cuda_1080p_5iterations.csv
```
