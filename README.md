# ROS IDL Buffer Benchmarks (`rosidl_buffer_benchmarks`)

Benchmarking and verification suite for ROS 2 `rosidl` buffer backends, comparing **Zero-Copy Transports (`shmem_buffer_backend`, `cuda_buffer_backend`)** against standard RMW wire serialization.

---

## 1. Repository Structure

```text
rosidl_buffer_benchmarks/
├── shmem_bb_benchmark_ws/          # Shared Memory (SHMEM) benchmark workspace
│   └── src/
│       ├── shmem_buffer_backend/   # Git submodule: shmem_buffer_backend
│       └── shmem_bb_benchmark/     # SHMEM nodes, launch files, run_benchmark.py
├── cuda_bb_benchmark_ws/           # CUDA IPC benchmark workspace
│   └── src/
│       ├── rosidl_buffer_backends/ # Git submodule: rosidl_buffer_backends
│       └── cuda_bb_benchmark/      # CUDA nodes, launch files, run_benchmark.py
└── README.md
```

- Launch files and detailed CSV schemas for each package:
  - [SHMEM Package & Launch Files Guide](shmem_bb_benchmark_ws/src/shmem_bb_benchmark/README.md)
  - [CUDA Package & Launch Files Guide](cuda_bb_benchmark_ws/src/cuda_bb_benchmark/README.md)

---

## 2. Prerequisites

- **Ubuntu 26.04 / 24.04** (`amd64` or `arm64`) with **ROS 2 Lyrical**
- **Supported RMW Implementations** (enforced by all launch files and `run_benchmark.py`):
  - `rmw_fastrtps_cpp` (default, tagged `_fastrtps.csv`)
  - `rmw_zenoh_cpp` (requires `rmw_zenohd`, tagged `_zenoh.csv`)
- **For CUDA Benchmarks**: NVIDIA GPU + Driver and CUDA Toolkit >= 11.8

---

## 3. Methodology Summary

- **Monotonic Timing**: Both publisher and subscriber sample `std::chrono::steady_clock` (`CLOCK_MONOTONIC`) right before `publish()` and at callback entry / buffer-ready completion.
- **Payload**: `sensor_msgs/msg/Image` with default `rgb8` encoding (3 bytes/pixel; e.g. `1920x1080` = `6,220,800` B) and byte-level content verification (`verify_content: true`).
- **Warm-Up Exclusion**: Initial `10` frames are excluded from latency, throughput, and resource metrics.
- **Pooled Statistics**: `run_benchmark.py` pools raw per-frame latency samples across all valid iterations to compute overall P50, P95, P99, Mean, and StdDev alongside per-run averages (`AVERAGE_PER_RUN` and `POOLED_RESULTS`).
- **Resource Monitoring**: Background `SystemMonitor` samples process CPU% (`(proc_cpu_time / wall_time) * 100 / online_cpus` every 200 ms) and PSS (`/proc/self/smaps_rollup` ~1 s), plus optional Jetson GPU/EMC telemetry for CUDA.

---

## 4. Build

Clone recursively (or initialize submodules with `git submodule update --init --recursive`):

```bash
# SHMEM workspace
cd shmem_bb_benchmark_ws
./install_deps.sh
./build.sh
source install/setup.bash
cd ..

# CUDA workspace (on CUDA-capable host)
cd cuda_bb_benchmark_ws
./install_deps.sh
./build.sh
source install/setup.bash
cd ..
```

---

## 5. Running Benchmarks (`run_benchmark.py`)

Both packages provide `run_benchmark.py` with identical CLI arguments:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `--resolution` | `1920x1080` | Image dimensions `WIDTHxHEIGHT` |
| `--rate` | `30.0` | Publishing rate in Hz |
| `--max-msgs` | `300` | Measured frames per iteration (`<= 0` uses default `300`) |
| `-i` / `--iterations` | `5` | Number of iterations per benchmark type |

### 5.1 SHMEM Benchmark Suite (`shmem_bb_benchmark`)

```bash
source shmem_bb_benchmark_ws/install/setup.bash

# Default run (1920x1080 @ 30 Hz, 300 msgs, 5 iterations)
ros2 run shmem_bb_benchmark run_benchmark.py

# Custom parameters
ros2 run shmem_bb_benchmark run_benchmark.py \
  --resolution 1920x1080 --rate 30.0 --max-msgs 300 --iterations 5
```

Runs all 4 cases and saves 4 CSVs in the current working directory (`shmem_<type>_<W>x<H>_<rate>hz_<max_msgs>msgs_<iterations>iter_<rmw>.csv`):

1. `cpu_to_cpu`: `cpu (pub) -> rmw -> cpu (sub)` (no SHMEM)
2. `shmem_to_shmem`: `shmem (pub) -> rmw -> shmem (sub)` (zero-copy)
3. `shmem_to_cpu`: `shmem (pub) -> rmw -> cpu (sub)` (CPU validation, no SHMEM copy)
4. `shmem_to_cpu_and_shmem`: `shmem (pub) -> rmw -> shmem + cpu (parallel)`

### 5.2 CUDA Benchmark Suite (`cuda_bb_benchmark`)

```bash
source cuda_bb_benchmark_ws/install/setup.bash

# Default run (1920x1080 @ 30 Hz, 300 msgs, 5 iterations)
ros2 run cuda_bb_benchmark run_benchmark.py

# Custom parameters (example with Zenoh RMW)
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run cuda_bb_benchmark run_benchmark.py \
  --resolution 1920x1080 --rate 30.0 --max-msgs 300 --iterations 5
```

Runs all 4 cases and saves 4 CSVs in the current working directory (`cuda_<type>_<W>x<H>_<rate>hz_<max_msgs>msgs_<iterations>iter_<rmw>.csv`):

1. `cpu_to_cpu`: `pub (gpu -> cpu) -> rmw -> sub (cpu -> gpu)` + GPU validation
2. `cuda_to_cuda`: `pub (gpu) -> rmw -> sub (gpu)` (zero-copy CUDA IPC) + GPU validation
3. `cuda_to_cpu`: `pub (gpu) -> rmw -> sub (cpu)` (CPU validation, no GPU copy)
4. `cuda_to_cpu_and_cuda`: `pub (gpu) -> rmw -> sub (gpu + cpu parallel)`

For individual launch files, node parameters, and CSV column schemas, see:
- [`shmem_bb_benchmark` README](shmem_bb_benchmark_ws/src/shmem_bb_benchmark/README.md)
- [`cuda_bb_benchmark` README](cuda_bb_benchmark_ws/src/cuda_bb_benchmark/README.md)
