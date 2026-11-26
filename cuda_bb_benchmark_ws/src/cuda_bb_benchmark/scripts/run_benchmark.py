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
Run the full CUDA benchmark suite across all 4 transport configurations:
  1. cpu_to_cpu          (standalone CPU -> CPU)
  2. cuda_to_cuda        (standalone CUDA -> CUDA)
  3. cuda_to_cpu         (standalone CUDA -> CPU fallback)
  4. cuda_to_cpu_and_cuda (parallel CUDA -> CPU + CUDA)

Each benchmark runs for the configured number of iterations and saves a unique
summary CSV (including per-iteration rows, AVERAGE_PER_RUN, and POOLED_RESULTS)
in the current working directory.
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


BENCHMARK_TYPES = [
    {
        'name': 'cpu_to_cpu',
        'label': 'CPU to CPU (Standalone)',
        'launch': 'cuda_or_wire_pub_sub.launch.py',
        'kind': 'single',
        'mode_arg': ('mode', 'wire'),
        'expect_cu': False,
        'expect_fb': True,
    },
    {
        'name': 'cuda_to_cuda',
        'label': 'CUDA to CUDA (Standalone)',
        'launch': 'cuda_or_wire_pub_sub.launch.py',
        'kind': 'single',
        'mode_arg': ('mode', 'cuda'),
        'expect_cu': True,
        'expect_fb': False,
    },
    {
        'name': 'cuda_to_cpu',
        'label': 'CUDA to CPU (Standalone)',
        'launch': 'cuda_fallback_two_subs.launch.py',
        'kind': 'two_subs',
        'mode_arg': ('sub_mode', 'cpu'),
        'expect_cu': False,
        'expect_fb': True,
    },
    {
        'name': 'cuda_to_cpu_and_cuda',
        'label': 'CUDA to CPU and CUDA (Parallel)',
        'launch': 'cuda_fallback_two_subs.launch.py',
        'kind': 'two_subs',
        'mode_arg': ('sub_mode', 'both'),
        'expect_cu': True,
        'expect_fb': True,
    },
]


def parse_resolution(res_str: str):
    """Parse 'WIDTHxHEIGHT' string into (width, height) integers."""
    try:
        parts = res_str.lower().split('x')
        if len(parts) == 2:
            w, h = int(parts[0]), int(parts[1])
            if w > 0 and h > 0:
                return w, h
    except Exception:
        pass
    raise argparse.ArgumentTypeError(
        f"Invalid resolution '{res_str}'. Expected format: WIDTHxHEIGHT (e.g. 1920x1080)")


def validate_iterations(val_str: str):
    try:
        val = int(val_str)
        if val >= 1:
            return val
    except ValueError:
        pass
    raise argparse.ArgumentTypeError(
        f"Invalid iteration count '{val_str}'. Must be an integer >= 1.")


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


def compute_average_row(rows, header, kind, is_both):
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

    if kind == 'two_subs':
        for key in ('mean_speedup', 'p50_speedup', 'p95_speedup', 'p99_speedup'):
            avg_row[key] = 'N/A'
        if is_both:
            for metric, sp_key in (
                ('mean_ms', 'mean_speedup'),
                ('p50_ms', 'p50_speedup'),
                ('p95_ms', 'p95_speedup'),
                ('p99_ms', 'p99_speedup'),
            ):
                try:
                    cu = float(avg_row[f'cuda_br_{metric}'])
                    fb = float(avg_row[f'fallback_{metric}'])
                    if cu > 0:
                        avg_row[sp_key] = f"{fb / cu:.2f}x"
                except (ValueError, TypeError, KeyError):
                    pass

    return avg_row


def build_pooled_row(avg_row, header, kind, cu_stats, fb_stats, is_both):
    pooled_row = {col: 'N/A' for col in header}
    for col in ('backend', 'resolution', 'size_bytes', 'target_hz'):
        if col in avg_row:
            pooled_row[col] = avg_row[col]

    if kind == 'single':
        st = cu_stats if cu_stats is not None else fb_stats
        if st is not None:
            pooled_row['br_p50_latency_ms'] = f"{st['p50']:.3f}"
            pooled_row['br_p95_latency_ms'] = f"{st['p95']:.3f}"
            pooled_row['br_p99_latency_ms'] = f"{st['p99']:.3f}"
            pooled_row['br_mean_latency_ms'] = f"{st['mean']:.3f}"
            pooled_row['br_min_latency_ms'] = f"{st['min']:.3f}"
            pooled_row['br_max_latency_ms'] = f"{st['max']:.3f}"
            pooled_row['br_std_dev_ms'] = f"{st['std']:.3f}"
        return pooled_row

    if cu_stats is not None:
        pooled_row['cuda_br_p50_ms'] = f"{cu_stats['p50']:.3f}"
        pooled_row['cuda_br_p95_ms'] = f"{cu_stats['p95']:.3f}"
        pooled_row['cuda_br_p99_ms'] = f"{cu_stats['p99']:.3f}"
        pooled_row['cuda_br_mean_ms'] = f"{cu_stats['mean']:.3f}"
        pooled_row['cuda_br_min_ms'] = f"{cu_stats['min']:.3f}"
        pooled_row['cuda_br_max_ms'] = f"{cu_stats['max']:.3f}"
        pooled_row['cuda_br_std_ms'] = f"{cu_stats['std']:.3f}"

    if fb_stats is not None:
        pooled_row['fallback_p50_ms'] = f"{fb_stats['p50']:.3f}"
        pooled_row['fallback_p95_ms'] = f"{fb_stats['p95']:.3f}"
        pooled_row['fallback_p99_ms'] = f"{fb_stats['p99']:.3f}"
        pooled_row['fallback_mean_ms'] = f"{fb_stats['mean']:.3f}"
        pooled_row['fallback_min_ms'] = f"{fb_stats['min']:.3f}"
        pooled_row['fallback_max_ms'] = f"{fb_stats['max']:.3f}"
        pooled_row['fallback_std_ms'] = f"{fb_stats['std']:.3f}"

    if is_both and cu_stats is not None and fb_stats is not None:
        if cu_stats['mean'] > 0:
            pooled_row['mean_speedup'] = f"{fb_stats['mean'] / cu_stats['mean']:.2f}x"
        if cu_stats['p50'] > 0:
            pooled_row['p50_speedup'] = f"{fb_stats['p50'] / cu_stats['p50']:.2f}x"
        if cu_stats['p95'] > 0:
            pooled_row['p95_speedup'] = f"{fb_stats['p95'] / cu_stats['p95']:.2f}x"
        if cu_stats['p99'] > 0:
            pooled_row['p99_speedup'] = f"{fb_stats['p99'] / cu_stats['p99']:.2f}x"

    return pooled_row


def run_benchmark_type(cfg, width, height, rate, max_msgs, iterations):
    b_name = cfg['name']
    b_label = cfg['label']
    kind = cfg['kind']
    expect_cu = cfg['expect_cu']
    expect_fb = cfg['expect_fb']
    is_both = expect_cu and expect_fb

    rate_tag = f"{rate:g}".replace('.', 'p')
    out_filename = f"cuda_{b_name}_{width}x{height}_{rate_tag}hz_{max_msgs}msgs_{iterations}iter.csv"
    out_path = Path.cwd() / out_filename

    print("\n" + "#" * 90)
    print(f" BENCHMARK: {b_label} ({b_name})")
    print(f" Resolution: {width}x{height} | Rate: {rate:.1f} Hz | Max Msgs: {max_msgs} | Iterations: {iterations}")
    print(f" Output CSV: {out_path}")
    print("#" * 90)

    valid_rows = []
    excluded_iterations = []
    pooled_cu_samples = []
    pooled_fb_samples = []
    header = None

    for it in range(1, iterations + 1):
        print(f"\n>>> [{b_name}] Iteration {it}/{iterations}...")
        with tempfile.NamedTemporaryFile(suffix=f'_{b_name}_it_{it}.csv', delete=False) as f:
            tmp_path = f.name

        single_lat_path = ''
        cu_lat_path = ''
        fb_lat_path = ''
        if kind == 'single':
            with tempfile.NamedTemporaryFile(suffix=f'_{b_name}_it_{it}_raw.csv', delete=False) as f:
                single_lat_path = f.name
        else:
            if expect_cu:
                with tempfile.NamedTemporaryFile(suffix=f'_{b_name}_it_{it}_cu_raw.csv', delete=False) as f:
                    cu_lat_path = f.name
            if expect_fb:
                with tempfile.NamedTemporaryFile(suffix=f'_{b_name}_it_{it}_fb_raw.csv', delete=False) as f:
                    fb_lat_path = f.name

        mode_key, mode_val = cfg['mode_arg']
        cmd = [
            'ros2', 'launch', 'cuda_bb_benchmark', cfg['launch'],
            f'width:={width}',
            f'height:={height}',
            f'rate:={rate:.1f}',
            f'max_msgs:={max_msgs}',
            f'{mode_key}:={mode_val}',
            f'csv_path:={tmp_path}',
        ]
        if single_lat_path:
            cmd.append(f'latency_csv_path:={single_lat_path}')
        if cu_lat_path:
            cmd.append(f'cuda_latency_csv_path:={cu_lat_path}')
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

            excl_reason = None
            iter_cu_samples = []
            iter_fb_samples = []

            if kind == 'single':
                recv_frames = parse_int(row.get('total_received_frames'))
                mean_ms = parse_float(row.get('br_mean_latency_ms'))
                total_raw, samples = load_raw_latency_csv(
                    single_lat_path, 'gpu_ready_latency_ms', 'gpu_op_ok')
                if not recv_frames or recv_frames <= 0 or mean_ms is None:
                    excl_reason = "subscriber incomplete (0 frames or missing metrics)"
                elif total_raw is None or total_raw != recv_frames:
                    excl_reason = f"raw sample count mismatch (raw={total_raw}, summary={recv_frames})"
                elif not samples:
                    excl_reason = "raw CSV has 0 valid GPU samples"
                else:
                    if expect_cu:
                        iter_cu_samples = samples
                    else:
                        iter_fb_samples = samples
            else:
                if expect_cu:
                    cu_frames = parse_int(row.get('cuda_received_frames'))
                    cu_mean = parse_float(row.get('cuda_br_mean_ms'))
                    total_raw, iter_cu_samples = load_raw_latency_csv(
                        cu_lat_path, 'gpu_ready_latency_ms', 'gpu_op_ok')
                    if not cu_frames or cu_frames <= 0 or cu_mean is None:
                        excl_reason = "CUDA subscriber incomplete (0 frames or missing metrics)"
                    elif total_raw is None or total_raw != cu_frames:
                        excl_reason = f"CUDA raw sample count mismatch (raw={total_raw}, summary={cu_frames})"
                    elif not iter_cu_samples:
                        excl_reason = "CUDA raw CSV has 0 valid GPU samples"

                if excl_reason is None and expect_fb:
                    fb_frames = parse_int(row.get('fallback_received_frames'))
                    fb_mean = parse_float(row.get('fallback_mean_ms'))
                    total_raw, iter_fb_samples = load_raw_latency_csv(
                        fb_lat_path, 'gpu_ready_latency_ms', 'gpu_op_ok')
                    if not fb_frames or fb_frames <= 0 or fb_mean is None:
                        excl_reason = "CPU fallback subscriber incomplete (0 frames or missing metrics)"
                    elif total_raw is None or total_raw != fb_frames:
                        excl_reason = f"CPU fallback raw sample count mismatch (raw={total_raw}, summary={fb_frames})"
                    elif not iter_fb_samples:
                        excl_reason = "CPU fallback raw CSV has 0 valid GPU samples"

                if excl_reason is None and not is_both:
                    row['mean_speedup'] = 'N/A'
                    row['p50_speedup'] = 'N/A'
                    row['p95_speedup'] = 'N/A'
                    row['p99_speedup'] = 'N/A'

            if excl_reason is not None:
                print(f"Excluding Iteration {it}: {excl_reason}")
                excluded_iterations.append((it, excl_reason))
                continue

            valid_rows.append(row)
            pooled_cu_samples.extend(iter_cu_samples)
            pooled_fb_samples.extend(iter_fb_samples)

        except subprocess.CalledProcessError as err:
            reason = f"launch exited with status {err.returncode}"
            print(f"Error during [{b_name}] iteration {it}: {err}")
            excluded_iterations.append((it, reason))
        finally:
            for p in (tmp_path, single_lat_path, cu_lat_path, fb_lat_path):
                if p and os.path.exists(p):
                    os.unlink(p)

        if it < iterations:
            time.sleep(1.0)

    if not valid_rows or not header:
        print(f"ERROR: [{b_name}] produced no valid iterations.")
        return {
            'name': b_name,
            'label': b_label,
            'ok': False,
            'csv_path': None,
            'valid_count': 0,
            'excluded': excluded_iterations,
            'cu_stats': None,
            'fb_stats': None,
        }

    cu_stats = calc_pooled_stats(pooled_cu_samples) if expect_cu else None
    fb_stats = calc_pooled_stats(pooled_fb_samples) if expect_fb else None

    avg_row = compute_average_row(valid_rows, header, kind, is_both)
    avg_row['iteration'] = 'AVERAGE_PER_RUN'

    pooled_row = build_pooled_row(avg_row, header, kind, cu_stats, fb_stats, is_both)
    pooled_row['iteration'] = 'POOLED_RESULTS'

    full_header = ['iteration'] + [h for h in header if h != 'iteration']
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=full_header)
        writer.writeheader()
        for r in valid_rows:
            writer.writerow({k: r.get(k, '') for k in full_header})
        writer.writerow({k: avg_row.get(k, '') for k in full_header})
        writer.writerow({k: pooled_row.get(k, '') for k in full_header})

    print(f"\n[{b_name}] Summary saved to: {out_path}")
    return {
        'name': b_name,
        'label': b_label,
        'ok': True,
        'csv_path': str(out_path),
        'valid_count': len(valid_rows),
        'excluded': excluded_iterations,
        'cu_stats': cu_stats,
        'fb_stats': fb_stats,
        'is_both': is_both,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run all 4 CUDA benchmark types and save uniquely named summary CSVs in CWD."
    )
    parser.add_argument(
        '--max-msgs', '--max_msgs', dest='max_msgs', type=int, default=300,
        help="Number of benchmark messages per iteration (default: 300)"
    )
    parser.add_argument(
        '--rate', type=float, default=30.0,
        help="Publishing rate in Hz (default: 30.0)"
    )
    parser.add_argument(
        '--resolution', type=parse_resolution, default='1920x1080',
        help="Image resolution WxH (default: 1920x1080)"
    )
    parser.add_argument(
        '-i', '--iterations', '--num-iterations', dest='iterations',
        type=validate_iterations, default=5,
        help="Number of iterations per benchmark type (default: 5)"
    )

    args = parser.parse_args()
    if args.max_msgs <= 0:
        args.max_msgs = 300
    width, height = args.resolution

    print("=" * 90)
    print(" CUDA BENCHMARK SUITE")
    print(f" Resolution: {width}x{height} | Rate: {args.rate:.1f} Hz | Max Msgs: {args.max_msgs} | Iterations: {args.iterations}")
    print(f" Output Directory: {Path.cwd()}")
    print("=" * 90)

    results = []
    for cfg in BENCHMARK_TYPES:
        res = run_benchmark_type(
            cfg, width, height, args.rate, args.max_msgs, args.iterations)
        results.append(res)
        time.sleep(1.0)

    print("\n" + "=" * 90)
    print(" CUDA BENCHMARK SUITE SUMMARY (POOLED RESULTS, BR = Buffer-Ready)")
    print("=" * 90)
    any_failed = False
    for res in results:
        if not res['ok']:
            any_failed = True
            print(f" [FAILED] {res['label']} | Excluded: {res['excluded']}")
            continue
        parts = []
        if res['cu_stats'] is not None:
            s = res['cu_stats']
            parts.append(
                f"CUDA BR(n={s['count']}): P50={s['p50']:.3f} P95={s['p95']:.3f} "
                f"P99={s['p99']:.3f} Mean={s['mean']:.3f} Std={s['std']:.3f} ms"
            )
        if res['fb_stats'] is not None:
            s = res['fb_stats']
            parts.append(
                f"CPU BR(n={s['count']}): P50={s['p50']:.3f} P95={s['p95']:.3f} "
                f"P99={s['p99']:.3f} Mean={s['mean']:.3f} Std={s['std']:.3f} ms"
            )
        if res.get('is_both') and res['cu_stats'] and res['fb_stats'] and res['cu_stats']['mean'] > 0:
            sp = res['fb_stats']['mean'] / res['cu_stats']['mean']
            parts.append(f"Speedup={sp:.2f}x")
        print(f" [{res['valid_count']}/{args.iterations} runs] {res['label']}")
        if res['excluded']:
            excl_str = '; '.join(f"iter {it}: {reason}" for it, reason in res['excluded'])
            print(f"   Excl : {excl_str}")
        print(f"   Stats: {' | '.join(parts)}")
        print(f"   CSV  : {res['csv_path']}")
    print("=" * 90 + "\n")

    if any_failed:
        sys.exit(1)


if __name__ == '__main__':
    main()
