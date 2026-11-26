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
Run repeated benchmark iterations for a single resolution and compute both
per-iteration averages and pooled raw-sample statistics.
"""

import argparse
import csv
import math
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


def format_loss_pair(val1, val2):
    s1 = format_val(val1, 2)
    s2 = format_val(val2, 2)
    p1 = f"{s1}%" if s1 != 'N/A' else 'N/A'
    p2 = f"{s2}%" if s2 != 'N/A' else 'N/A'
    return f"{p1} / {p2}"


def parse_float(val):
    if val is None or val == 'N/A' or val == '':
        return None
    try:
        return float(val)
    except (ValueError, TypeError):
        return None


def parse_int(val):
    if val is None or val == 'N/A' or val == '':
        return None
    try:
        return int(float(val))
    except (ValueError, TypeError):
        return None


def calc_pooled_stats(samples):
    """Compute P50, P95, P99, mean, and population std dev matching C++ calc_stats."""
    if not samples:
        return None
    s = sorted(samples)
    n = len(s)

    def pct(p):
        if n == 1:
            return s[0]
        rank = p * (n - 1)
        lower = int(rank)
        upper = min(lower + 1, n - 1)
        weight = rank - lower
        return s[lower] * (1.0 - weight) + s[upper] * weight

    mean_v = sum(s) / n
    sq_diff = sum((v - mean_v) ** 2 for v in s)
    std_v = math.sqrt(sq_diff / n)
    return {
        'count': n,
        'p50': pct(0.50),
        'p95': pct(0.95),
        'p99': pct(0.99),
        'mean': mean_v,
        'std': std_v,
        'min': s[0],
        'max': s[-1],
    }


def load_raw_latency_csv(path, value_col, ok_col):
    """Load raw per-frame latency CSV and return (total_rows, valid_samples)."""
    if not path or not os.path.exists(path):
        return None, []
    total_rows = 0
    valid_samples = []
    try:
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                total_rows += 1
                if row.get(ok_col, '').strip() == '1':
                    val = parse_float(row.get(value_col))
                    if val is not None:
                        valid_samples.append(val)
    except Exception:
        return None, []
    return total_rows, valid_samples


def compute_average_row(rows, header, sub_mode):
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
            if 'fps' in col or 'pct' in col or 'util' in col or 'pss' in col or 'mb' in col:
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

    # Only compute speedup when both subscribers ran in the same iteration mode
    for key in ('mean_speedup', 'p50_speedup', 'p99_speedup'):
        avg_row[key] = 'N/A'
    if sub_mode == 'both':
        try:
            shm = float(avg_row['shmem_mean_ms'])
            fb = float(avg_row['fallback_mean_ms'])
            if shm > 0:
                avg_row['mean_speedup'] = f"{fb / shm:.2f}x"
        except (ValueError, TypeError, KeyError):
            pass
        try:
            shm = float(avg_row['shmem_p50_ms'])
            fb = float(avg_row['fallback_p50_ms'])
            if shm > 0:
                avg_row['p50_speedup'] = f"{fb / shm:.2f}x"
        except (ValueError, TypeError, KeyError):
            pass
        try:
            shm = float(avg_row['shmem_p99_ms'])
            fb = float(avg_row['fallback_p99_ms'])
            if shm > 0:
                avg_row['p99_speedup'] = f"{fb / shm:.2f}x"
        except (ValueError, TypeError, KeyError):
            pass

    return avg_row


def build_pooled_row(avg_row, header, shm_stats, fb_stats, sub_mode):
    pooled_row = {col: 'N/A' for col in header}
    for col in ('resolution', 'size_bytes', 'target_hz'):
        if col in avg_row:
            pooled_row[col] = avg_row[col]

    if shm_stats is not None:
        pooled_row['shmem_p50_ms'] = f"{shm_stats['p50']:.3f}"
        pooled_row['shmem_p95_ms'] = f"{shm_stats['p95']:.3f}"
        pooled_row['shmem_p99_ms'] = f"{shm_stats['p99']:.3f}"
        pooled_row['shmem_mean_ms'] = f"{shm_stats['mean']:.3f}"
        pooled_row['shmem_min_ms'] = f"{shm_stats['min']:.3f}"
        pooled_row['shmem_max_ms'] = f"{shm_stats['max']:.3f}"
        pooled_row['shmem_std_ms'] = f"{shm_stats['std']:.3f}"

    if fb_stats is not None:
        pooled_row['fallback_p50_ms'] = f"{fb_stats['p50']:.3f}"
        pooled_row['fallback_p95_ms'] = f"{fb_stats['p95']:.3f}"
        pooled_row['fallback_p99_ms'] = f"{fb_stats['p99']:.3f}"
        pooled_row['fallback_mean_ms'] = f"{fb_stats['mean']:.3f}"
        pooled_row['fallback_min_ms'] = f"{fb_stats['min']:.3f}"
        pooled_row['fallback_max_ms'] = f"{fb_stats['max']:.3f}"
        pooled_row['fallback_std_ms'] = f"{fb_stats['std']:.3f}"

    if sub_mode == 'both' and shm_stats is not None and fb_stats is not None:
        if shm_stats['mean'] > 0:
            pooled_row['mean_speedup'] = f"{fb_stats['mean'] / shm_stats['mean']:.2f}x"
        if shm_stats['p50'] > 0:
            pooled_row['p50_speedup'] = f"{fb_stats['p50'] / shm_stats['p50']:.2f}x"
        if shm_stats['p99'] > 0:
            pooled_row['p99_speedup'] = f"{fb_stats['p99'] / shm_stats['p99']:.2f}x"

    return pooled_row


def print_comparison_table(
    rows, avg_row, pooled_row, shm_stats, fb_stats,
    excluded_iterations, width, height, rate, sub_mode,
):
    sep = "=" * 212
    dash = "-" * 212
    print("\n" + sep)
    print(f" SHMEM ITERATION RESULTS & POOLED SUMMARY [ {width}x{height} @ {rate:.1f} Hz | sub_mode={sub_mode} ]")
    print(sep)

    headers = [
        ("Run", 16),
        ("Pub FPS", 8),
        ("Shm FPS", 8),
        ("CPU FPS", 8),
        ("Loss% (Shm/CPU)", 17),
        ("Shm P50 (ms)", 12),
        ("CPU P50 (ms)", 12),
        ("Shm P95 (ms)", 12),
        ("CPU P95 (ms)", 12),
        ("Shm P99 (ms)", 12),
        ("CPU P99 (ms)", 12),
        ("Shm Mean (ms)", 13),
        ("CPU Mean (ms)", 13),
        ("Shm Std (ms)", 12),
        ("CPU Std (ms)", 12),
        ("Speedup", 8),
    ]

    print(" | ".join(f"{h:<{w}}" for h, w in headers))
    print(dash)

    def print_row(label, r, is_pooled=False):
        pub_fps = 'N/A' if is_pooled else format_val(r.get('actual_pub_fps'), 2)
        shm_fps = 'N/A' if is_pooled else format_val(r.get('shmem_recv_fps'), 2)
        fb_fps = 'N/A' if is_pooled else format_val(r.get('fallback_recv_fps'), 2)
        loss_str = 'N/A' if is_pooled else format_loss_pair(
            r.get('shmem_loss_pct'), r.get('fallback_loss_pct'))
        speedup = r.get('mean_speedup', 'N/A') if sub_mode == 'both' else 'N/A'

        row_vals = [
            (label, 16),
            (pub_fps, 8),
            (shm_fps, 8),
            (fb_fps, 8),
            (loss_str, 17),
            (format_val(r.get('shmem_p50_ms'), 3), 12),
            (format_val(r.get('fallback_p50_ms'), 3), 12),
            (format_val(r.get('shmem_p95_ms'), 3), 12),
            (format_val(r.get('fallback_p95_ms'), 3), 12),
            (format_val(r.get('shmem_p99_ms'), 3), 12),
            (format_val(r.get('fallback_p99_ms'), 3), 12),
            (format_val(r.get('shmem_mean_ms'), 3), 13),
            (format_val(r.get('fallback_mean_ms'), 3), 13),
            (format_val(r.get('shmem_std_ms'), 3), 12),
            (format_val(r.get('fallback_std_ms'), 3), 12),
            (speedup, 8),
        ]
        print(" | ".join(f"{v:<{w}}" for v, w in row_vals))

    for r in rows:
        print_row(f"Iteration {r.get('iteration', '?')}", r)

    print(dash)
    print_row("AVERAGE PER RUN", avg_row)
    print_row("POOLED RESULTS", pooled_row, is_pooled=True)
    print(sep)

    # Pooled summary, variability, sample validation, and exclusions
    print("\n--- POOLED RESULTS & VARIABILITY SUMMARY ---")
    print(f" Valid iterations included : {len(rows)} | Excluded: {len(excluded_iterations)}")
    if excluded_iterations:
        for ex_it, ex_reason in excluded_iterations:
            print(f"   - Excluded Iteration {ex_it}: {ex_reason}")

    if shm_stats is not None:
        shm_means = [parse_float(r.get('shmem_mean_ms')) for r in rows]
        shm_means = [m for m in shm_means if m is not None]
        var_str = f"{min(shm_means):.3f} - {max(shm_means):.3f} ms" if shm_means else "N/A"
        print(
            f" SHMEM Pooled (n={shm_stats['count']} valid samples, warm-up excluded): "
            f"P50={shm_stats['p50']:.3f} ms | P95={shm_stats['p95']:.3f} ms | "
            f"P99={shm_stats['p99']:.3f} ms | Mean={shm_stats['mean']:.3f} ms | "
            f"Std={shm_stats['std']:.3f} ms | Per-Run Mean Range: {var_str}"
        )

    if fb_stats is not None:
        fb_means = [parse_float(r.get('fallback_mean_ms')) for r in rows]
        fb_means = [m for m in fb_means if m is not None]
        var_str = f"{min(fb_means):.3f} - {max(fb_means):.3f} ms" if fb_means else "N/A"
        print(
            f" CPU   Pooled (n={fb_stats['count']} valid samples, warm-up excluded): "
            f"P50={fb_stats['p50']:.3f} ms | P95={fb_stats['p95']:.3f} ms | "
            f"P99={fb_stats['p99']:.3f} ms | Mean={fb_stats['mean']:.3f} ms | "
            f"Std={fb_stats['std']:.3f} ms | Per-Run Mean Range: {var_str}"
        )

    if sub_mode == 'both' and shm_stats is not None and fb_stats is not None and shm_stats['mean'] > 0:
        pooled_speedup = fb_stats['mean'] / shm_stats['mean']
        reduction_pct = (
            (fb_stats['mean'] - shm_stats['mean']) / fb_stats['mean'] * 100.0
            if fb_stats['mean'] > 0 else 0.0
        )
        iter_speedups = []
        for r in rows:
            sp_str = str(r.get('mean_speedup', '')).rstrip('x')
            sp_val = parse_float(sp_str)
            if sp_val is not None:
                iter_speedups.append(sp_val)
        sp_range = f"{min(iter_speedups):.2f}x - {max(iter_speedups):.2f}x" if iter_speedups else "N/A"
        print(
            f" Overall Speedup (CPU pooled mean / SHMEM pooled mean): {pooled_speedup:.2f}x | "
            f"Latency Reduction: {reduction_pct:.2f}% | Per-Run Speedup Range: {sp_range}"
        )
    else:
        print(" Overall Speedup / Latency Reduction: N/A (only computed when sub_mode=both)")


def main():
    parser = argparse.ArgumentParser(
        description="Run repeated benchmark iterations for single resolution and compute averages and pooled stats."
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
        '--sub-mode', choices=['both', 'shmem', 'cpu'], default='both',
        help="Subscribers to run: 'both', 'shmem', or 'cpu' (default: both)"
    )
    parser.add_argument(
        '-o', '--output', type=str, default='',
        help="Path to save all iteration results, average per run, and pooled results CSV"
    )

    args = parser.parse_args()
    width, height = args.resolution

    expect_shm = args.sub_mode in ('both', 'shmem')
    expect_fb = args.sub_mode in ('both', 'cpu')

    print("=" * 80)
    print(f" BENCHMARK ITERATION RUNNER: {args.iterations} iteration(s)")
    print(f" Resolution: {width}x{height} | Target Rate: {args.rate:.1f} Hz | Messages: {args.max_msgs}")
    print(f" Subscriber Mode: {args.sub_mode}")
    print("=" * 80)

    valid_rows = []
    excluded_iterations = []
    pooled_shm_samples = []
    pooled_fb_samples = []
    header = None

    for it in range(1, args.iterations + 1):
        print(f"\n>>> Running Iteration {it}/{args.iterations}...")
        with tempfile.NamedTemporaryFile(suffix=f'_it_{it}.csv', delete=False) as tmp_file:
            tmp_path = tmp_file.name
        shm_lat_path = ''
        fb_lat_path = ''
        if expect_shm:
            with tempfile.NamedTemporaryFile(suffix=f'_it_{it}_shm_raw.csv', delete=False) as f:
                shm_lat_path = f.name
        if expect_fb:
            with tempfile.NamedTemporaryFile(suffix=f'_it_{it}_fb_raw.csv', delete=False) as f:
                fb_lat_path = f.name

        cmd = [
            'ros2', 'launch', 'shmem_bb_benchmark', 'shmem_fallback_two_subs.launch.py',
            f'width:={width}',
            f'height:={height}',
            f'rate:={args.rate:.1f}',
            f'max_msgs:={args.max_msgs}',
            f'sub_mode:={args.sub_mode}',
            'verify_content:=true',
            f'csv_path:={tmp_path}',
        ]
        if shm_lat_path:
            cmd.append(f'shmem_latency_csv_path:={shm_lat_path}')
        if fb_lat_path:
            cmd.append(f'fallback_latency_csv_path:={fb_lat_path}')

        try:
            subprocess.run(cmd, check=True)

            if not os.path.exists(tmp_path):
                excluded_iterations.append((it, "summary CSV missing"))
                continue

            with open(tmp_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                header = reader.fieldnames
                iter_rows = list(reader)

            if not iter_rows:
                excluded_iterations.append((it, "summary CSV empty"))
                continue

            row = dict(iter_rows[-1])
            row['iteration'] = str(it)

            # Validate completeness and raw sample counts for active subscribers
            excl_reason = None
            iter_shm_samples = []
            iter_fb_samples = []

            if expect_shm:
                shm_frames = parse_int(row.get('shmem_received_frames'))
                shm_mean = parse_float(row.get('shmem_mean_ms'))
                total_raw, iter_shm_samples = load_raw_latency_csv(
                    shm_lat_path, 'callback_latency_ms', 'data_access_ok')
                if not shm_frames or shm_frames <= 0 or shm_mean is None:
                    excl_reason = "SHMEM subscriber incomplete (0 frames or missing metrics)"
                elif total_raw is None or total_raw != shm_frames:
                    excl_reason = (
                        f"SHMEM raw sample count mismatch "
                        f"(raw={total_raw}, summary={shm_frames})"
                    )
                elif not iter_shm_samples:
                    excl_reason = "SHMEM raw CSV has 0 valid samples"

            if excl_reason is None and expect_fb:
                fb_frames = parse_int(row.get('fallback_received_frames'))
                fb_mean = parse_float(row.get('fallback_mean_ms'))
                total_raw, iter_fb_samples = load_raw_latency_csv(
                    fb_lat_path, 'callback_latency_ms', 'data_access_ok')
                if not fb_frames or fb_frames <= 0 or fb_mean is None:
                    excl_reason = "CPU fallback subscriber incomplete (0 frames or missing metrics)"
                elif total_raw is None or total_raw != fb_frames:
                    excl_reason = (
                        f"CPU fallback raw sample count mismatch "
                        f"(raw={total_raw}, summary={fb_frames})"
                    )
                elif not iter_fb_samples:
                    excl_reason = "CPU fallback raw CSV has 0 valid samples"

            if excl_reason is not None:
                print(f"Excluding Iteration {it}: {excl_reason}")
                excluded_iterations.append((it, excl_reason))
                continue

            if args.sub_mode != 'both':
                row['mean_speedup'] = 'N/A'
                row['p50_speedup'] = 'N/A'
                row['p99_speedup'] = 'N/A'

            valid_rows.append(row)
            pooled_shm_samples.extend(iter_shm_samples)
            pooled_fb_samples.extend(iter_fb_samples)

        except subprocess.CalledProcessError as err:
            reason = f"launch exited with status {err.returncode}"
            print(f"Error during iteration {it}: {err}")
            excluded_iterations.append((it, reason))
        finally:
            for p in (tmp_path, shm_lat_path, fb_lat_path):
                if p and os.path.exists(p):
                    os.unlink(p)

        if it < args.iterations:
            time.sleep(1.0)

    if not valid_rows:
        print("Error: No valid iteration benchmark results collected.")
        if excluded_iterations:
            for ex_it, ex_reason in excluded_iterations:
                print(f"  - Iteration {ex_it}: {ex_reason}")
        sys.exit(1)

    shm_stats = calc_pooled_stats(pooled_shm_samples) if expect_shm else None
    fb_stats = calc_pooled_stats(pooled_fb_samples) if expect_fb else None

    avg_row = compute_average_row(valid_rows, header, args.sub_mode)
    avg_row['iteration'] = 'AVERAGE_PER_RUN'

    pooled_row = build_pooled_row(avg_row, header, shm_stats, fb_stats, args.sub_mode)
    pooled_row['iteration'] = 'POOLED_RESULTS'

    print_comparison_table(
        valid_rows, avg_row, pooled_row, shm_stats, fb_stats,
        excluded_iterations, width, height, args.rate, args.sub_mode,
    )

    if args.output:
        out_path = Path(args.output).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        full_header = ['iteration'] + [h for h in header if h != 'iteration']
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=full_header)
            writer.writeheader()
            for r in valid_rows:
                writer.writerow({k: r.get(k, '') for k in full_header})
            writer.writerow({k: avg_row.get(k, '') for k in full_header})
            writer.writerow({k: pooled_row.get(k, '') for k in full_header})
        print(f"\nAll iteration outputs, per-run average, and pooled results written to: {out_path}\n")


if __name__ == '__main__':
    main()
