# cuda_bb_benchmark

ROS 2 benchmarking and verification package for [`cuda_buffer_backend`](../rosidl_buffer_backends/cuda_buffer_backend) and [`cuda_buffer`](../rosidl_buffer_backends/cuda_buffer).

---

## 1. Transport Cases

| Case | Launch Configuration | Path |
| :--- | :--- | :--- |
| `cpu_to_cpu` | `cuda_or_wire_pub_sub.launch.py mode:=wire` | `pub (gpu -> cpu) -> rmw -> sub (cpu -> gpu)` + GPU validation |
| `cuda_to_cuda` | `cuda_or_wire_pub_sub.launch.py mode:=cuda` | `pub (gpu) -> rmw -> sub (gpu)` (zero-copy CUDA IPC) + GPU validation |
| `cuda_to_cpu` | `cuda_fallback_two_subs.launch.py sub_mode:=cpu` | `pub (gpu) -> rmw -> sub (cpu)` (CPU validation, no GPU copy) |
| `cuda_to_cpu_and_cuda` | `cuda_fallback_two_subs.launch.py sub_mode:=both` | `pub (gpu) -> rmw -> sub (gpu + cpu parallel)` |

---

## 2. Launch Files

> **RMW requirement**: All launch files validate `RMW_IMPLEMENTATION` and support only `rmw_fastrtps_cpp` (`_fastrtps.csv`) and `rmw_zenoh_cpp` (`_zenoh.csv`).

### 2.1 `cuda_fallback_two_subs.launch.py`
Runs 1 CUDA publisher (`use_cuda:=true`) with configurable subscribers via `sub_mode`:
- `sub_mode:=both` (default): `cuda_subscriber` (zero-copy CUDA IPC) + `fallback_subscriber` (pure CPU fallback, no H2D copy)
- `sub_mode:=cuda`: only `cuda_subscriber`
- `sub_mode:=cpu`: only `fallback_subscriber`

```bash
source install/setup.bash

# Parallel CUDA + CPU fallback (1000 benchmark frames + 10 warm-up)
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=both

# CUDA -> CPU only
ros2 launch cuda_bb_benchmark cuda_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=cpu
```

**Arguments**: `width` (`1920`), `height` (`1080`), `rate` (`30.0`), `max_msgs` (`1000`, `<= 0` falls back to `1000`), `warmup_msgs` (`10`), `history_depth` (`80`), `cuda_device_id` (`0`), `sub_mode` (`both`), `csv_path` (`/tmp/cuda_fallback_comparison_<rmw>.csv`), `cuda_latency_csv_path` (`""`), `fallback_latency_csv_path` (`""`).

### 2.2 `cuda_or_wire_pub_sub.launch.py`
Runs 1 publisher and 1 subscriber in separate processes:

```bash
# Zero-copy CUDA IPC
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000

# Wire mode: pub (gpu -> cpu) -> rmw -> sub (cpu -> gpu)
ros2 launch cuda_bb_benchmark cuda_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000
```

**Arguments**: `width` (`1920`), `height` (`1080`), `rate` (`30.0`), `mode` (`cuda`), `max_msgs` (`1000`, `<= 0` falls back to `1000`), `warmup_msgs` (`10`), `history_depth` (`80`), `cuda_device_id` (`0`), `csv_path` (`/tmp/cuda_mode_results_<rmw>.csv`), `latency_csv_path` (`""`).

---

## 3. Suite Runner (`run_benchmark.py`)

Runs all 4 CUDA transport cases (`cpu_to_cpu`, `cuda_to_cuda`, `cuda_to_cpu`, `cuda_to_cpu_and_cuda`) for `iterations` runs and writes one CSV per case (including per-iteration rows, `AVERAGE_PER_RUN`, and `POOLED_RESULTS`) in the current working directory:

```bash
ros2 run cuda_bb_benchmark run_benchmark.py \
  --resolution 1920x1080 --rate 30.0 --max-msgs 300 --iterations 5
```

---

## 4. CSV Schemas

- **Two-subscriber (`cuda_fallback_two_subs.launch.py`)**: `resolution, size_bytes, target_hz, actual_pub_fps, cuda_recv_fps, fallback_recv_fps, total_published_frames, cuda_received_frames, fallback_received_frames, cuda_dropped_frames, fallback_dropped_frames, cuda_loss_pct, fallback_loss_pct, cuda_cb_*, cuda_br_*, fallback_*, p50_speedup, p95_speedup, p99_speedup, mean_speedup, cuda_device_op_ms, fallback_device_op_ms, pub_cpu_*, cuda_sub_cpu_*, fallback_sub_cpu_*, pub_pss_*, cuda_sub_pss_*, fallback_sub_pss_*, gpu_util_pct, gpu_freq_mhz, emc_util_pct, emc_freq_mhz`.
- **Single-mode (`cuda_or_wire_pub_sub.launch.py`)**: `backend, size_bytes, resolution, target_hz, actual_pub_fps, actual_recv_fps, total_published_frames, total_received_frames, dropped_frames, frame_loss_pct, failed_gpu_ops_count, failed_content_verify_count, cb_*, br_*, device_op_mean_ms, pub_cpu_*, sub_cpu_*, pub_pss_*, sub_pss_*, gpu_util_pct, gpu_freq_mhz, emc_util_pct, emc_freq_mhz`.
