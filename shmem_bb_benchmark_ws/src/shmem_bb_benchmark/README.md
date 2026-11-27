# shmem_bb_benchmark

ROS 2 benchmarking and verification package for [`shmem_buffer_backend`](../shmem_buffer_backend) and [`shmem_buffer`](../shmem_buffer).

---

## 1. Nodes

### `shmem_image_publisher_node`
- Publishes `sensor_msgs::msg::Image` (`rgb8`, 3 bytes/pixel by default) using `shmem_buffer::allocate_buffer()` + `shmem_buffer::from_output_buffer()` when `use_shm:=true`, or standard CPU `rosidl::Buffer<uint8_t>` when `use_shm:=false`.
- **Key parameters**: `topic` (`"test_shmem_image"`), `rate` (`10.0`), `width` (`640`), `height` (`480`), `encoding` (`"rgb8"`), `history_depth` (`max(80, rate * 2.5)`), `max_messages` (`0`), `warmup_messages` (`10`), `use_shm` (`true`), `auto_exit` (`false`).

### `shmem_image_subscriber_node`
- Subscribes with `acceptable_buffer_backends = "shmem"` when `use_shm && require_shmem`.
- Acquires `read_handle->get_ptr()` for `"shmem"` or `msg->data.data()` for `"cpu"`, verifies payload bytes (`verify_content:=true`), and logs callback-entry and data-ready latencies plus process CPU% and PSS.

---

## 2. Launch Files

> **RMW requirement**: All launch files validate `RMW_IMPLEMENTATION` and support only `rmw_fastrtps_cpp` (`_fastrtps.csv`) and `rmw_zenoh_cpp` (`_zenoh.csv`).

### 2.1 `shmem_fallback_two_subs.launch.py`
Runs 1 SHMEM publisher (`use_shm:=true`) with configurable subscribers via `sub_mode`:
- `sub_mode:=both` (default): `subscriber_shared` (zero-copy SHMEM) + `subscriber_fallback` (CPU wire)
- `sub_mode:=shmem`: only `subscriber_shared`
- `sub_mode:=cpu`: only `subscriber_fallback`

```bash
source install/setup.bash

# Parallel SHMEM + CPU fallback (1000 benchmark frames + 10 warm-up)
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=both

# SHMEM -> CPU only
ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
  width:=1920 height:=1080 rate:=30.0 max_msgs:=1000 sub_mode:=cpu
```

**Arguments**: `width` (`1920`), `height` (`1080`), `rate` (`30.0`), `max_msgs` (`1000`, `<= 0` falls back to `1000`), `warmup_msgs` (`10`), `history_depth` (`80`), `sub_mode` (`both`), `csv_path` (`/tmp/shmem_fallback_comparison_<rmw>.csv`), `shmem_latency_csv_path` (`""`), `fallback_latency_csv_path` (`""`).

### 2.2 `shmem_or_wire_pub_sub.launch.py`
Runs 1 publisher and 1 subscriber in separate processes in either `shared` (SHMEM zero-copy) or `wire` (CPU) mode:

```bash
# Zero-copy SHMEM -> SHMEM
ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=shared max_msgs:=1000

# Standard CPU -> CPU over RMW
ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
  width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000
```

**Arguments**: `width` (`1920`), `height` (`1080`), `rate` (`30.0`), `mode` (`shared`), `max_msgs` (`1000`, `<= 0` falls back to `1000`), `warmup_msgs` (`10`), `history_depth` (`80`), `csv_path` (`/tmp/shmem_mode_results_<rmw>.csv`), `latency_csv_path` (`""`).

---

## 3. Suite Runner (`run_benchmark.py`)

Runs all 4 SHMEM transport cases (`cpu_to_cpu`, `shmem_to_shmem`, `shmem_to_cpu`, `shmem_to_cpu_and_shmem`) for `iterations` runs and writes one CSV per case (including per-iteration rows, `AVERAGE_PER_RUN`, and `POOLED_RESULTS`) in the current working directory:

```bash
ros2 run shmem_bb_benchmark run_benchmark.py \
  --resolution 1920x1080 --rate 30.0 --max-msgs 300 --iterations 5
```

---

## 4. CSV Schemas

- **Two-subscriber (`shmem_fallback_two_subs.launch.py`)**: `resolution, size_bytes, target_hz, actual_pub_fps, shmem_recv_fps, fallback_recv_fps, total_published_frames, shmem_received_frames, fallback_received_frames, shmem_dropped_frames, fallback_dropped_frames, shmem_loss_pct, fallback_loss_pct, shmem_p50_ms, fallback_p50_ms, p50_speedup, shmem_p95_ms, fallback_p95_ms, shmem_p99_ms, fallback_p99_ms, p99_speedup, shmem_mean_ms, fallback_mean_ms, mean_speedup, shmem_min_ms, fallback_min_ms, shmem_max_ms, fallback_max_ms, shmem_std_ms, fallback_std_ms, shmem_outliers, fallback_outliers, shmem_dr_*, fallback_dr_*, pub_cpu_*, shmem_sub_cpu_*, fallback_sub_cpu_*, pub_pss_*, shmem_sub_pss_*, fallback_sub_pss_*`.
- **Single-mode (`shmem_or_wire_pub_sub.launch.py`)**: `backend, size_bytes, resolution, target_hz, actual_pub_fps, actual_recv_fps, total_published_frames, total_received_frames, dropped_frames, frame_loss_pct, failed_data_access_count, failed_content_verify_count, mean_latency_ms, min_latency_ms, p50_latency_ms, p95_latency_ms, p99_latency_ms, max_latency_ms, std_dev_ms, outliers_3sigma_count, data_ready_*, pub_cpu_*, sub_cpu_*, pub_pss_*, sub_pss_*`.
