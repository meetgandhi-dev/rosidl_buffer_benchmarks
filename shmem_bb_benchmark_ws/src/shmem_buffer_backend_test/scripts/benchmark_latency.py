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

"""Automated benchmark tool measuring latency of shmem vs non-shm across sizes and rates."""

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
        f"Invalid resolution '{res_str}'. Expected format: WIDTHxHEIGHT (e.g. 640x480)")


def run_single_test(
    width: int,
    height: int,
    rate_hz: float,
    max_frames: int,
    output_csv: Path,
    use_shm: bool = True,
    domain_id: int = 42,
    timeout_sec: float = 30.0,
):
    """Run publisher and subscriber nodes for one benchmark configuration."""
    backend_tag = 'shmem' if use_shm else 'cpu'
    topic = f'benchmark_image_{backend_tag}_{width}_{height}_{int(rate_hz)}'
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(domain_id)
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    sub_cmd = [
        'ros2', 'run', 'shmem_buffer_backend_test', 'shmem_image_subscriber_node',
        '--ros-args',
        '-p', f'topic:={topic}',
        '-p', f'max_messages:={max_frames}',
        '-p', 'auto_exit:=true',
        '-p', f'target_hz:={rate_hz}',
        '-p', f'summary_csv_path:={output_csv.resolve()}',
        '-p', f'use_shm:={str(use_shm).lower()}',
        '-p', f'require_shmem:={str(use_shm).lower()}',
        '-p', 'verify_content:=true',
    ]

    pub_cmd = [
        'ros2', 'run', 'shmem_buffer_backend_test', 'shmem_image_publisher_node',
        '--ros-args',
        '-p', f'topic:={topic}',
        '-p', f'width:={width}',
        '-p', f'height:={height}',
        '-p', f'rate:={rate_hz}',
        '-p', f'max_messages:={max_frames}',
        '-p', 'auto_exit:=true',
        '-p', f'use_shm:={str(use_shm).lower()}',
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
        description='Benchmark shmem vs non-shm (CPU) latency across sizes and rates.')
    parser.add_argument(
        '--sizes', nargs='+', default=['640x480', '1280x720', '1920x1080'],
        help='Resolutions to test, e.g. 640x480 1280x720 1920x1080')
    parser.add_argument(
        '--rates', nargs='+', type=float, default=[10.0, 30.0, 60.0],
        help='Publishing rates in Hz to test, e.g. 10 30 60')
    parser.add_argument(
        '--max-frames', type=int, default=50,
        help='Cap for maximum frames to send per test (default: 50)')
    parser.add_argument(
        '--output', type=str, default='shmem_vs_non_shm_latency.csv',
        help='CSV output file path (default: shmem_vs_non_shm_latency.csv)')
    parser.add_argument(
        '--backends', nargs='+', choices=['shmem', 'cpu', 'both'], default=['both'],
        help="Backends to benchmark: 'shmem', 'cpu', or 'both' (default: both)")
    args = parser.parse_args()

    parsed_resolutions = [parse_resolution(s) for s in args.sizes]
    output_path = Path(args.output).resolve()

    if 'both' in args.backends:
        backends_to_test = [True, False]
    else:
        backends_to_test = [b == 'shmem' for b in args.backends]

    if output_path.exists():
        output_path.unlink()

    total_tests = len(parsed_resolutions) * len(args.rates) * len(backends_to_test)
    test_idx = 0

    print('=' * 88)
    print(f' STARTING LATENCY BENCHMARK: SHMEM vs NON-SHM ({total_tests} configurations)')
    print(f' Frame cap: {args.max_frames} frames per run')
    print(f' Output CSV: {output_path}')
    print('=' * 88)

    base_domain_id = 50 + (os.getpid() % 40)
    for w, h in parsed_resolutions:
        payload_bytes = w * h
        human_size = f'{payload_bytes / (1024 * 1024):.2f} MB' \
            if payload_bytes >= 1024 * 1024 else f'{payload_bytes / 1024:.1f} KB'

        for rate in args.rates:
            for use_shm in backends_to_test:
                test_idx += 1
                b_name = 'shmem' if use_shm else 'non-shm (cpu)'
                print(f'[{test_idx}/{total_tests}] Testing {b_name} @ {w}x{h} ({human_size}), '
                      f'{rate:.0f} Hz (cap: {args.max_frames} frames)...', flush=True)

                run_single_test(
                    width=w,
                    height=h,
                    rate_hz=rate,
                    max_frames=args.max_frames,
                    output_csv=output_path,
                    use_shm=use_shm,
                    domain_id=base_domain_id + (test_idx % 25),
                    timeout_sec=max(15.0, (args.max_frames / max(1.0, rate)) * 2.5 + 8.0),
                )

    print('\n' + '=' * 88)
    print(' BENCHMARK COMPLETED — COMPARISON TABLE (SHMEM VS NON-SHM)')
    print('=' * 88)

    if not output_path.exists():
        print('Error: Benchmark results file was not created.')
        sys.exit(1)

    # Read CSV content
    with open(output_path, 'r', newline='') as f:
        reader = list(csv.reader(f))

    if not reader:
        print('Error: Results CSV is empty.')
        sys.exit(1)

    header = reader[0]
    rows = reader[1:]

    # Print formatted Markdown table
    header_str = ' | '.join(header)
    separator_str = ' | '.join(['---'] * len(header))
    print(f'| {header_str} |')
    print(f'| {separator_str} |')
    for row in rows:
        print(f"| {' | '.join(row)} |")

    # Side-by-side comparison if both were tested
    if len(backends_to_test) > 1:
        print('\n' + '=' * 88)
        print(' SIDE-BY-SIDE SUMMARY: SHMEM vs NON-SHM')
        print('=' * 88)
        print(f"{'Resolution':<12} | {'Size':<10} | {'Hz':<6} | "
              f"{'shmem (ms)':<12} | {'non-shm (ms)':<14} | {'Speedup / Improvement'}")
        print('-' * 88)

        by_key = {}
        for r in rows:
            # columns: backend, size_bytes, res, target_hz, ..., avg_latency_ms
            b, sz, res, hz, avg_lat = r[0], r[1], r[2], r[3], r[6]
            key = (res, hz)
            if key not in by_key:
                by_key[key] = {'size': sz}
            by_key[key][b] = float(avg_lat)

        for (res, hz), data in by_key.items():
            shm_val = data.get('shmem')
            cpu_val = data.get('cpu')
            sz_int = int(data['size'])
            human_sz = f'{sz_int / (1024 * 1024):.2f} MB' \
                if sz_int >= 1024 * 1024 else f'{sz_int / 1024:.0f} KB'

            if shm_val is not None and cpu_val is not None:
                diff_ms = cpu_val - shm_val
                pct = ((cpu_val - shm_val) / cpu_val) * 100.0 if cpu_val > 0 else 0.0
                ratio = cpu_val / shm_val if shm_val > 0 else 1.0
                speedup_str = f'{ratio:.2f}x faster ({pct:.1f}% less latency, -{diff_ms:.2f} ms)'
                print(f'{res:<12} | {human_sz:<10} | {float(hz):<6.0f} | '
                      f'{shm_val:<12.3f} | {cpu_val:<14.3f} | {speedup_str}')

    print('\n' + '=' * 88)
    print(' RAW CSV FORMAT:')
    print('=' * 88)
    with open(output_path, 'r') as f:
        print(f.read().strip())
    print('=' * 88)
    print(f'Results successfully saved to: {output_path}')


if __name__ == '__main__':
    main()
