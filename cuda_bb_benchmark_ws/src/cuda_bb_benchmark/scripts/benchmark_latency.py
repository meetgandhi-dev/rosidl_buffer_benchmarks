#!/usr/bin/env python3
# Copyright 2026 Meet Gandhi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Automated benchmark tool measuring latency of CUDA zero-copy vs wire across sizes and rates."""

import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import time


def parse_resolution(res_str: str):
    """Parse 'WIDTHxHEIGHT' string into (width, height) integers."""
    try:
        parts = res_str.lower().split('x')
        if len(parts) == 2:
            return int(parts[0]), int(parts[1])
    except Exception:
        pass
    raise argparse.ArgumentTypeError(
        f"Invalid resolution '{res_str}'. Expected format: WIDTHxHEIGHT (e.g. 1920x1080)")


def run_single_test(
    width: int,
    height: int,
    rate_hz: float,
    max_frames: int,
    output_csv: Path,
    use_cuda: bool = True,
    warmup_frames: int = 10,
    domain_id: int = 42,
    timeout_sec: float = 60.0,
):
    """Run publisher and subscriber nodes for one benchmark configuration."""
    backend_tag = 'cuda' if use_cuda else 'cpu'
    topic = f'benchmark_image_{backend_tag}_{width}_{height}_{int(rate_hz)}'
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    sub_cmd = [
        'ros2', 'run', 'cuda_bb_benchmark', 'cuda_image_subscriber_node',
        '--ros-args',
        '-p', f'topic:={topic}',
        '-p', f'max_messages:={max_frames}',
        '-p', f'warmup_messages:={warmup_frames}',
        '-p', 'auto_exit:=true',
        '-p', f'target_hz:={rate_hz}',
        '-p', f'summary_csv_path:={output_csv.resolve()}',
        '-p', f'use_cuda:={str(use_cuda).lower()}',
        '-p', f'require_cuda:={str(use_cuda).lower()}',
        '-p', 'verify_content:=false',
    ]

    pub_cmd = [
        'ros2', 'run', 'cuda_bb_benchmark', 'cuda_image_publisher_node',
        '--ros-args',
        '-p', f'topic:={topic}',
        '-p', f'width:={width}',
        '-p', f'height:={height}',
        '-p', f'rate:={rate_hz}',
        '-p', f'max_messages:={max_frames}',
        '-p', f'warmup_messages:={warmup_frames}',
        '-p', 'auto_exit:=true',
        '-p', f'use_cuda:={str(use_cuda).lower()}',
        '-p', 'wait_for_subscriber:=true',
    ]

    # Start subscriber first
    sub_proc = subprocess.Popen(sub_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.8)

    # Start publisher
    pub_proc = subprocess.Popen(pub_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if sub_proc.poll() is not None and pub_proc.poll() is not None:
            break
        time.sleep(0.1)

    if sub_proc.poll() is None:
        sub_proc.terminate()
        try:
            sub_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            sub_proc.kill()

    if pub_proc.poll() is None:
        pub_proc.terminate()
        try:
            pub_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pub_proc.kill()


def main():
    parser = argparse.ArgumentParser(
        description='Benchmark CUDA zero-copy vs wire (CPU) latency across sizes and rates.')
    parser.add_argument(
        '--sizes', nargs='+', default=['640x480', '1280x720', '1920x1080', '3840x2160'],
        help='Resolutions to test, e.g. 640x480 1280x720 1920x1080 3840x2160')
    parser.add_argument(
        '--rates', nargs='+', type=float, default=[10.0, 30.0, 60.0],
        help='Publishing rates in Hz to test, e.g. 10 30 60')
    parser.add_argument(
        '--max-frames', type=int, default=100,
        help='Cap for measured benchmark frames per run (default: 100)')
    parser.add_argument(
        '--warmup-frames', type=int, default=10,
        help='Number of initial frames to ignore for setup (default: 10)')
    parser.add_argument(
        '--output', type=str, default='cuda_vs_wire_latency.csv',
        help='CSV output file path (default: cuda_vs_wire_latency.csv)')
    parser.add_argument(
        '--backends', nargs='+', choices=['cuda', 'cpu', 'both'], default=['both'],
        help="Backends to benchmark: 'cuda', 'cpu', or 'both' (default: both)")
    args = parser.parse_args()

    parsed_resolutions = [parse_resolution(s) for s in args.sizes]
    output_path = Path(args.output).resolve()

    if 'both' in args.backends:
        backends_to_test = [True, False]
    else:
        backends_to_test = [b == 'cuda' for b in args.backends]

    if output_path.exists():
        output_path.unlink()

    total_tests = len(parsed_resolutions) * len(args.rates) * len(backends_to_test)
    test_idx = 0

    print('=' * 88)
    print(f' STARTING LATENCY BENCHMARK: CUDA ZERO-COPY vs WIRE ({total_tests} configurations)')
    print(f' Frame cap: {args.max_frames} measured frames per run (+ {args.warmup_frames} warm-up frames)')
    print(f' Output CSV: {output_path}')
    print('=' * 88)

    base_domain_id = 60 + (os.getpid() % 30)

    for width, height in parsed_resolutions:
        for rate in args.rates:
            for use_cuda in backends_to_test:
                test_idx += 1
                mode_str = 'CUDA (Zero-Copy)' if use_cuda else 'CPU (Wire Copy)'
                payload_mb = (width * height) / (1024 * 1024)

                print(
                    f'[{test_idx}/{total_tests}] Testing {width}x{height} ({payload_mb:.2f} MB) '
                    f'@ {rate:.0f} Hz | Backend: {mode_str} ...',
                    flush=True)

                current_domain = base_domain_id + (test_idx % 20)
                run_single_test(
                    width=width,
                    height=height,
                    rate_hz=rate,
                    max_frames=args.max_frames,
                    warmup_frames=args.warmup_frames,
                    output_csv=output_path,
                    use_cuda=use_cuda,
                    domain_id=current_domain,
                )
                time.sleep(0.5)

    print('=' * 88)
    print(' BENCHMARK SWEEP COMPLETED SUCCESSFULLY!')
    print(f' Results written to: {output_path}')
    print('=' * 88)

    if output_path.exists():
        try:
            with open(output_path, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                if rows:
                    print('\n--- SUMMARY RESULTS TABLE ---')
                    print(f'{"Backend":<10} | {"Resolution":<12} | {"Size (B)":<10} | {"Target Hz":<10} | '
                          f'{"Avg (ms)":<10} | {"Min (ms)":<10} | {"Max (ms)":<10} | {"StdDev (ms)":<12}')
                    print('-' * 95)
                    for r in rows:
                        print(f'{r.get("backend",""):<10} | {r.get("resolution",""):<12} | {r.get("size_bytes",""):<10} | '
                              f'{float(r.get("target_hz",0)):<10.1f} | {float(r.get("avg_latency_ms",0)):<10.3f} | '
                              f'{float(r.get("min_latency_ms",0)):<10.3f} | {float(r.get("max_latency_ms",0)):<10.3f} | '
                              f'{float(r.get("std_dev_ms",0)):<12.3f}')
        except Exception as e:
            print(f'Could not format summary table: {e}')


if __name__ == '__main__':
    main()
