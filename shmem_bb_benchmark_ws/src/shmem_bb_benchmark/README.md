# shmem_bb_benchmark

A comprehensive ROS 2 test and verification package for [`shmem_buffer_backend`](../shmem_buffer_backend) and [`shmem_buffer`](../shmem_buffer).

This package provides standalone executable nodes, composable components, launch files, unit tests (GTest & pytest), and integration launch tests (`launch_testing`) to test and demonstrate zero-copy shared-memory transport for `rosidl::Buffer<uint8_t>` in ROS 2 (e.g. `sensor_msgs/msg/Image`).

---

## Features & Included Nodes

### 1. `shmem_image_publisher_node`
- Publishes `sensor_msgs::msg::Image` messages whose payload is allocated in shared memory using `shmem_buffer::allocate_buffer()`.
- Obtains exclusive zero-copy write access via `shmem_buffer::from_output_buffer()`.
- Writes deterministic test patterns for data verification.
- Implements a configurable message history ring buffer to maintain references to active pool blocks while in flight.
- **Parameters**:
  - `topic` (`string`, default: `"test_shmem_image"`): Output image topic.
  - `rate` (`double`, default: `10.0`): Publishing rate in Hz.
  - `width` (`int`, default: `640`): Frame width.
  - `height` (`int`, default: `480`): Frame height.
  - `encoding` (`string`, default: `"mono8"`): Image encoding.
  - `history_depth` (`int`, default: `20`): Number of published messages kept alive in pool history.
  - `max_messages` (`int`, default: `0`): Number of messages to publish before stopping (0 = infinite).
  - `wait_for_subscriber` (`bool`, default: `false`): Wait until at least one subscriber is connected before publishing.

### 2. `shmem_image_subscriber_node`
- Subscribes to `sensor_msgs::msg::Image` with `subscription_options.acceptable_buffer_backends = "shmem"` to enforce zero-copy shared-memory transport and reject fallback to CPU copies.
- Verifies that `msg->data.get_backend_type() == "shmem"`.
- Reads buffer data using `shmem_buffer::from_input_buffer()` without copying.
- Validates payload integrity against the deterministic test pattern.
- Computes message transmission latency.
- Publishes diagnostic validation topics (`<result_prefix>/count`, `<result_prefix>/all_valid`, `<result_prefix>/backend_valid`, `<result_prefix>/content_valid`).
- **Parameters**:
  - `topic` (`string`, default: `"test_shmem_image"`): Input image topic.
  - `result_prefix` (`string`, default: `"shmem_test"`): Prefix for status and diagnostic topics.
  - `require_shmem` (`bool`, default: `true`): Require shared-memory backend.
  - `verify_content` (`bool`, default: `true`): Verify payload data byte-by-byte.
  - `publish_diagnostics` (`bool`, default: `true`): Publish validation status topics.

---

## Building and Running

### Build

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select shmem_bb_benchmark
```

### Run Launch Files Interactively

1. **Mixed Publisher + 2 Subscribers (Shared + Fallback)**:
   Runs 3 nodes: 1 shared publisher, 1 shared subscriber (zero-copy), and 1 fallback subscriber (standard DDS wire copy) simultaneously on the same topic. Automatically runs for `max_msgs` frames (default: 1000) and displays merged comparison results in both a formatted terminal table and a merged CSV format comparing both subscribers:
   ```bash
   source install/setup.bash
   # Run 1000 frames (default) and output merged comparison CSV:
   ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
     width:=1920 height:=1080 rate:=30.0

   # Or customize frame count and output path:
   ros2 launch shmem_bb_benchmark shmem_fallback_two_subs.launch.py \
     width:=1920 height:=1080 rate:=30.0 max_msgs:=500 csv_path:=/tmp/comparison.csv
   ```
   **Merged CSV Schema**:
   `resolution,size_bytes,target_hz,frame_cap,shmem_frames,fallback_frames,shmem_avg_latency_ms,fallback_avg_latency_ms,latency_diff_ms,speedup,shmem_min_latency_ms,fallback_min_latency_ms,shmem_max_latency_ms,fallback_max_latency_ms,shmem_std_dev_ms,fallback_std_dev_ms`

2. **Selectable Mode Pub/Sub (Shared vs Wire)**:
   Runs 2 nodes (1 publisher and 1 subscriber) in either `shared` (zero-copy SHM) or `wire` (standard DDS CPU copy) mode. Automatically runs for `max_msgs` frames (default: 1000) and displays final results in CSV format:
   ```bash
   source install/setup.bash
   # Run in Shared Memory mode:
   ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
     width:=1920 height:=1080 rate:=30.0 mode:=shared max_msgs:=1000

   # Run in Wire / Fallback mode:
   ros2 launch shmem_bb_benchmark shmem_or_wire_pub_sub.launch.py \
     width:=1920 height:=1080 rate:=30.0 mode:=wire max_msgs:=1000
   ```

### Run Standalone Nodes Manually

Terminal 1 (Subscriber):
```bash
source install/setup.bash
ros2 run shmem_bb_benchmark shmem_image_subscriber_node
```

Terminal 2 (Publisher):
```bash
source install/setup.bash
ros2 run shmem_bb_benchmark shmem_image_publisher_node --ros-args -p rate:=20.0
```

---

## Iteration Benchmarking: Single Resolution Repeated Runs

To run repeated benchmark iterations for a single resolution and compute averages across runs:

```bash
source install/setup.bash
ros2 run shmem_bb_benchmark run_iterations.py \
  --iterations 5 \
  --resolution 1920x1080 \
  --rate 30.0 \
  --max-msgs 1000 \
  --sub-mode both \
  --output shmem_1080p_5iterations.csv
```

You can also run nodes individually with non-shm mode:
```bash
# Non-shm subscriber
ros2 run shmem_bb_benchmark shmem_image_subscriber_node --ros-args -p use_shm:=false

# Non-shm publisher
ros2 run shmem_bb_benchmark shmem_image_publisher_node --ros-args -p use_shm:=false
```
