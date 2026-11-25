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

"""
Run repeated benchmark iterations for a single resolution and calculate iteration averages for CUDA buffer backend.
"""

import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
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


def validate_iterations(val_str: str):
    try:
        val = int(val_str)
        if 1 <= val <= 10:
            return val
    except ValueError:
        pass
    raise argparse.ArgumentTypeError(
        f"Invalid iteration '{val_str}'. Must be an integer between 1 and 10.")


def format_val(val, decimals=2):
    if val is None or val == 'N/A' or val == '':
        return 'N/A'
    try:
        num = float(val)
        return f"{num:.{decimals}f}"
    except (ValueError, TypeError):
        return str(val)


def compute_average_row(rows, header):
    avg_row = {}
    for col in header:
        values = []
        is_numeric = True
        for r in rows:
            v = r.get(col, '')
            if v == 'N/A' or v == '' or 'x' in str(v).lower():
                is_numeric = False
                break
            try:
                values.append(float(v))
            except ValueError:
                is_numeric = False
                break

        if is_numeric and values:
            mean_v = sum(values) / len(values)
            if 'fps' in col or 'pct' in col or 'util' in col or 'ram' in col or 'rss' in col or 'temp' in col:
                avg_row[col] = f"{mean_v:.2f}"
            elif 'ms' in col:
                avg_row[col] = f"{mean_v:.3f}"
            elif 'frames' in col or 'count' in col or 'outliers' in col:
                avg_row[col] = f"{mean_v:.1f}"
            else:
                avg_row[col] = f"{mean_v:.2f}"
        else:
            sample_val = rows[0].get(col, 'N/A') if rows else 'N/A'
            if all(r.get(col) == sample_val for r in rows):
                avg_row[col] = sample_val
            else:
                avg_row[col] = 'N/A'

    # Recalculate speedup ratios if available
    try:
        if 'cuda_br_mean_ms' in avg_row and 'fallback_mean_ms' in avg_row:
            cu = float(avg_row['cuda_br_mean_ms'])
            fb = float(avg_row['fallback_mean_ms'])
            if cu > 0:
                avg_row['mean_speedup'] = f"{fb / cu:.2f}x"
        if 'cuda_br_p50_ms' in avg_row and 'fallback_p50_ms' in avg_row:
            cu = float(avg_row['cuda_br_p50_ms'])
            fb = float(avg_row['fallback_p50_ms'])
            if cu > 0:
                avg_row['p50_speedup'] = f"{fb / cu:.2f}x"
        if 'cuda_br_p99_ms' in avg_row and 'fallback_p99_ms' in avg_row:
            cu = float(avg_row['cuda_br_p99_ms'])
            fb = float(avg_row['fallback_p99_ms'])
            if cu > 0:
                avg_row['p99_speedup'] = f"{fb / cu:.2f}x"
    except (ValueError, TypeError, KeyError):
        pass

    return avg_row


def print_comparison_table(rows, avg_row, width, height, rate, sub_mode):
    print("\n" + "=" * 175)
    print(f" CUDA ITERATION RESULTS & AVERAGE SUMMARY [ {width}x{height} @ {rate:.1f} Hz | sub_mode={sub_mode} ]")
    print("=" * 175)

    headers = [
        ("Run", 12),
        ("Pub FPS", 9),
        ("CUDA FPS", 9),
        ("CPU FPS", 9),
        ("Loss% (CU/CPU)", 16),
        ("CU BR P50", 10),
        ("CPU P50", 9),
        ("CU BR P95", 10),
        ("CPU P95", 9),
        ("CU BR P99", 10),
        ("CPU P99", 9),
        ("CU BR Mean", 11),
        ("CPU Mean", 10),
        ("CU BR Std", 10),
        ("CPU Std", 9),
        ("Speedup", 8),
    ]

    header_line = " | ".join(f"{h:<{w}}" for h, w in headers)
    print(header_line)
    print("-" * 175)

    for idx, r in enumerate(rows, 1):
        run_name = f"Iteration {idx}"
        pub_fps = format_val(r.get('actual_pub_fps'), 2)
        cu_fps = format_val(r.get('cuda_recv_fps'), 2)
        fb_fps = format_val(r.get('fallback_recv_fps'), 2)
        cu_loss = format_val(r.get('cuda_loss_pct'), 2)
        fb_loss = format_val(r.get('fallback_loss_pct'), 2)
        loss_str = f"{cu_loss}% / {fb_loss}%"

        cu_br_p50 = format_val(r.get('cuda_br_p50_ms'), 3)
        fb_p50 = format_val(r.get('fallback_p50_ms'), 3)
        cu_br_p95 = format_val(r.get('cuda_br_p95_ms'), 3)
        fb_p95 = format_val(r.get('fallback_p95_ms'), 3)
        cu_br_p99 = format_val(r.get('cuda_br_p99_ms'), 3)
        fb_p99 = format_val(r.get('fallback_p99_ms'), 3)
        cu_br_mean = format_val(r.get('cuda_br_mean_ms'), 3)
        fb_mean = format_val(r.get('fallback_mean_ms'), 3)
        cu_br_std = format_val(r.get('cuda_br_std_ms'), 3)
        fb_std = format_val(r.get('fallback_std_ms'), 3)
        speedup = r.get('mean_speedup', 'N/A')

        row_vals = [
            (run_name, 12),
            (pub_fps, 9),
            (cu_fps, 9),
            (fb_fps, 9),
            (loss_str, 16),
            (cu_br_p50, 10),
            (fb_p50, 9),
            (cu_br_p95, 10),
            (fb_p95, 9),
            (cu_br_p99, 10),
            (fb_p99, 9),
            (cu_br_mean, 11),
            (fb_mean, 10),
            (cu_br_std, 10),
            (fb_std, 9),
            (speedup, 8),
        ]
        print(" | ".join(f"{v:<{w}}" for v, w in row_vals))

    print("-" * 175)
    # Average row
    avg_cu_loss = format_val(avg_row.get('cuda_loss_pct'), 2)
    avg_fb_loss = format_val(avg_row.get('fallback_loss_pct'), 2)
    avg_loss_str = f"{avg_cu_loss}% / {avg_fb_loss}%"

    avg_vals = [
        ("AVERAGE", 12),
        (format_val(avg_row.get('actual_pub_fps'), 2), 9),
        (format_val(avg_row.get('cuda_recv_fps'), 2), 9),
        (format_val(avg_row.get('fallback_recv_fps'), 2), 9),
        (avg_loss_str, 16),
        (format_val(avg_row.get('cuda_br_p50_ms'), 3), 10),
        (format_val(avg_row.get('fallback_p50_ms'), 3), 9),
        (format_val(avg_row.get('cuda_br_p95_ms'), 3), 10),
        (format_val(avg_row.get('fallback_p95_ms'), 3), 9),
        (format_val(avg_row.get('cuda_br_p99_ms'), 3), 10),
        (format_val(avg_row.get('fallback_p99_ms'), 3), 9),
        (format_val(avg_row.get('cuda_br_mean_ms'), 3), 11),
        (format_val(avg_row.get('fallback_mean_ms'), 3), 10),
        (format_val(avg_row.get('cuda_br_std_ms'), 3), 10),
        (format_val(avg_row.get('fallback_std_ms'), 3), 9),
        (avg_row.get('mean_speedup', 'N/A'), 8),
    ]
    print(" | ".join(f"{v:<{w}}" for v, w in avg_vals))
    print("=" * 175)


def main():
    parser = argparse.ArgumentParser(
        description="Run repeated CUDA benchmark iterations for single resolution and compute averages."
    )
    parser.add_argument(
        '-i', '--iterations', type=validate_iterations, default=1,
        help="Number of times to iterate the benchmark (min: 1, max: 10; default: 1)"
    )
    parser.add_argument(
        '--resolution', type=parse_resolution, default='1920x1080',
        help="Single resolution to benchmark, e.g. 1920x1080 (default: 1920x1080)"
    )
    parser.add_argument(
        '--rate', type=float, default=30.0,
        help="Publishing rate in Hz (default: 30.0)"
    )
    parser.add_argument(
        '--max-msgs', type=int, default=1000,
        help="Number of messages to publish per iteration (default: 1000)"
    )
    parser.add_argument(
        '--sub-mode', choices=['both', 'cuda', 'cpu'], default='both',
        help="Subscribers to run: 'both', 'cuda', or 'cpu' (default: both)"
    )
    parser.add_argument(
        '-o', '--output', type=str, default='',
        help="Path to save all iteration results and average CSV"
    )

    args = parser.parse_args()
    width, height = args.resolution

    print("=" * 80)
    print(f" CUDA BENCHMARK ITERATION RUNNER: {args.iterations} iteration(s)")
    print(f" Resolution: {width}x{height} | Target Rate: {args.rate:.1f} Hz | Messages: {args.max_msgs}")
    print(f" Subscriber Mode: {args.sub_mode}")
    print("=" * 80)

    all_rows = []
    header = None

    for it in range(1, args.iterations + 1):
        print(f"\n>>> Running Iteration {it}/{args.iterations}...")
        with tempfile.NamedTemporaryFile(suffix=f'_it_{it}.csv', delete=False) as tmp_file:
            tmp_path = tmp_file.name

        cmd = [
            'ros2', 'launch', 'cuda_bb_benchmark', 'cuda_fallback_two_subs.launch.py',
            f'width:={width}',
            f'height:={height}',
            f'rate:={args.rate:.1f}',
            f'max_msgs:={args.max_msgs}',
            f'sub_mode:={args.sub_mode}',
            'verify_content:=true',
            f'csv_path:={tmp_path}',
        ]

        try:
            res = subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as err:
            print(f"Error during iteration {it}: {err}")
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            continue

        if os.path.exists(tmp_path):
            with open(tmp_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                header = reader.fieldnames
                for r in reader:
                    r_with_iter = dict(r)
                    r_with_iter['iteration'] = str(it)
                    all_rows.append(r_with_iter)
            os.unlink(tmp_path)

        if it < args.iterations:
            time.sleep(1.0)

    if not all_rows:
        print("Error: No iteration benchmark results collected.")
        sys.exit(1)

    avg_row = compute_average_row(all_rows, header)
    avg_row['iteration'] = 'AVERAGE'

    print_comparison_table(all_rows, avg_row, width, height, args.rate, args.sub_mode)

    if args.output:
        out_path = Path(args.output).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        full_header = ['iteration'] + [h for h in header if h != 'iteration']
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=full_header)
            writer.writeheader()
            for r in all_rows:
                writer.writerow({k: r.get(k, '') for k in full_header})
            writer.writerow({k: avg_row.get(k, '') for k in full_header})
        print(f"\nAll iteration outputs and average written to: {out_path}\n")


if __name__ == '__main__':
    main()
