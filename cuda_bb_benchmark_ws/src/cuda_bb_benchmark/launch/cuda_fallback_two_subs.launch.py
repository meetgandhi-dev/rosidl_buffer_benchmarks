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

import csv
import os
import sys
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def init_csv_file(context):
    """Clean up any stale temporary files from previous runs."""
    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
            for ext in ['.cuda.tmp', '.fallback.tmp', '.pub.tmp']:
                tmp_path = f'{csv_file}{ext}'
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
        except OSError:
            pass


def merge_and_print_csv_results(*args, **kwargs):
    """Merge cuda and fallback results into a single comparison CSV and print summary."""
    context = None
    for a in args:
        if hasattr(a, 'launch_configurations'):
            context = a
            break
    if context is None:
        return

    csv_file = context.launch_configurations.get('csv_path', '')
    if not csv_file:
        return

    cuda_tmp = f'{csv_file}.cuda.tmp'
    fallback_tmp = f'{csv_file}.fallback.tmp'
    pub_tmp = f'{csv_file}.pub.tmp'

    sub_mode = context.launch_configurations.get('sub_mode', 'both').strip().lower()
    expect_cuda = sub_mode in ['both', 'cuda', 'cu', 'gpu', 'all']
    expect_fallback = sub_mode in ['both', 'cpu', 'fallback', 'wire', 'all']

    # Wait briefly for active subscriber and publisher nodes to finalize writing summary files
    deadline = time.time() + 3.0
    while time.time() < deadline:
        cuda_ready = (not expect_cuda) or os.path.exists(cuda_tmp)
        fallback_ready = (not expect_fallback) or os.path.exists(fallback_tmp)
        if cuda_ready and fallback_ready:
            time.sleep(0.1)
            break
        time.sleep(0.1)

    def read_summary(path):
        if not os.path.exists(path):
            return None
        try:
            with open(path, 'r', newline='') as f:
                reader = list(csv.DictReader(f))
                if reader:
                    return reader[-1]
        except Exception:
            pass
        return None

    cuda_data = read_summary(cuda_tmp) if expect_cuda else None
    fallback_data = read_summary(fallback_tmp) if expect_fallback else None
    pub_data = read_summary(pub_tmp)

    # Clean up temporary files
    for tmp in [cuda_tmp, fallback_tmp, pub_tmp]:
        if os.path.exists(tmp):
            try:
                os.remove(tmp)
            except OSError:
                pass

    if not cuda_data and not fallback_data:
        return

    src = cuda_data or fallback_data
    width = context.launch_configurations.get('width', '')
    height = context.launch_configurations.get('height', '')
    res_fallback = f'{width}x{height}' if (width and height) else 'unknown'
    resolution = src.get('resolution', res_fallback)
    size_bytes = src.get('size_bytes', '')
    target_hz = src.get('target_hz', context.launch_configurations.get('rate', ''))

    actual_pub_fps = pub_data.get('actual_pub_fps', src.get('actual_pub_fps', src.get('actual_pub_hz', target_hz))) if pub_data else src.get('actual_pub_fps', src.get('actual_pub_hz', target_hz))
    total_published = pub_data.get('total_published_frames', src.get('total_published_frames', src.get('total_sent', 'N/A'))) if pub_data else src.get('total_published_frames', src.get('total_sent', 'N/A'))

    cuda_recv_fps = cuda_data.get('actual_recv_fps', cuda_data.get('actual_recv_hz', '0.0')) if cuda_data else 'N/A'
    fallback_recv_fps = fallback_data.get('actual_recv_fps', fallback_data.get('actual_recv_hz', '0.0')) if fallback_data else 'N/A'

    cuda_frames = cuda_data.get('total_received_frames', cuda_data.get('received_frames', '0')) if cuda_data else 'N/A'
    fallback_frames = fallback_data.get('total_received_frames', fallback_data.get('received_frames', '0')) if fallback_data else 'N/A'

    cuda_lost = cuda_data.get('dropped_frames', cuda_data.get('lost_frames', '0')) if cuda_data else 'N/A'
    fallback_lost = fallback_data.get('dropped_frames', fallback_data.get('lost_frames', '0')) if fallback_data else 'N/A'

    cuda_loss_pct = cuda_data.get('frame_loss_pct', cuda_data.get('loss_pct', '0.0')) if cuda_data else 'N/A'
    fallback_loss_pct = fallback_data.get('frame_loss_pct', fallback_data.get('loss_pct', '0.0')) if fallback_data else 'N/A'

    # Callback entry latencies
    cuda_cb_p50 = cuda_data.get('cb_p50_latency_ms', cuda_data.get('p50_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_p95 = cuda_data.get('cb_p95_latency_ms', cuda_data.get('p95_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_p99 = cuda_data.get('cb_p99_latency_ms', cuda_data.get('p99_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_mean = cuda_data.get('cb_mean_latency_ms', cuda_data.get('mean_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_min = cuda_data.get('cb_min_latency_ms', cuda_data.get('min_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_max = cuda_data.get('cb_max_latency_ms', cuda_data.get('max_latency_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_std = cuda_data.get('cb_std_dev_ms', cuda_data.get('std_dev_ms', '0.0')) if cuda_data else 'N/A'
    cuda_cb_outliers = cuda_data.get('cb_outliers_3sigma_count', cuda_data.get('outliers_3sigma_count', '0')) if cuda_data else 'N/A'

    # Buffer-ready latencies (after CUDA sync)
    cuda_br_p50 = cuda_data.get('br_p50_latency_ms', cuda_cb_p50) if cuda_data else 'N/A'
    cuda_br_p95 = cuda_data.get('br_p95_latency_ms', cuda_cb_p95) if cuda_data else 'N/A'
    cuda_br_p99 = cuda_data.get('br_p99_latency_ms', cuda_cb_p99) if cuda_data else 'N/A'
    cuda_br_mean = cuda_data.get('br_mean_latency_ms', cuda_cb_mean) if cuda_data else 'N/A'
    cuda_br_min = cuda_data.get('br_min_latency_ms', cuda_cb_min) if cuda_data else 'N/A'
    cuda_br_max = cuda_data.get('br_max_latency_ms', cuda_cb_max) if cuda_data else 'N/A'
    cuda_br_std = cuda_data.get('br_std_dev_ms', cuda_cb_std) if cuda_data else 'N/A'
    cuda_br_outliers = cuda_data.get('br_outliers_3sigma_count', cuda_cb_outliers) if cuda_data else 'N/A'

    # Fallback latencies (wire / CPU copy)
    fallback_p50 = fallback_data.get('br_p50_latency_ms', fallback_data.get('p50_latency_ms', '0.0')) if fallback_data else 'N/A'
    fallback_p95 = fallback_data.get('br_p95_latency_ms', fallback_data.get('p95_latency_ms', '0.0')) if fallback_data else 'N/A'
    fallback_p99 = fallback_data.get('br_p99_latency_ms', fallback_data.get('p99_latency_ms', '0.0')) if fallback_data else 'N/A'
    fallback_mean = fallback_data.get('br_mean_latency_ms', fallback_data.get('mean_latency_ms', fallback_data.get('avg_latency_ms', '0.0'))) if fallback_data else 'N/A'
    fallback_min = fallback_data.get('br_min_latency_ms', fallback_data.get('min_latency_ms', '0.0')) if fallback_data else 'N/A'
    fallback_max = fallback_data.get('br_max_latency_ms', fallback_data.get('max_latency_ms', '0.0')) if fallback_data else 'N/A'
    fallback_std = fallback_data.get('br_std_dev_ms', fallback_data.get('std_dev_ms', '0.0')) if fallback_data else 'N/A'
    fallback_outliers = fallback_data.get('br_outliers_3sigma_count', fallback_data.get('outliers_3sigma_count', '0')) if fallback_data else 'N/A'

    def calc_speedup(fb_str, cu_str):
        try:
            fb = float(fb_str)
            cu = float(cu_str)
            if cu > 0.0:
                return f'{fb / cu:.2f}x'
        except (ValueError, TypeError):
            pass
        return 'N/A'

    p50_speedup = calc_speedup(fallback_p50, cuda_br_p50)
    p95_speedup = calc_speedup(fallback_p95, cuda_br_p95)
    p99_speedup = calc_speedup(fallback_p99, cuda_br_p99)
    mean_speedup = calc_speedup(fallback_mean, cuda_br_mean)

    # CUDA import/sync vs CPU H2D overhead
    cuda_device_op = cuda_data.get('device_op_mean_ms', 'N/A') if cuda_data else 'N/A'
    fallback_device_op = fallback_data.get('device_op_mean_ms', 'N/A') if fallback_data else 'N/A'

    # Resource telemetry
    pub_cpu_mean = pub_data.get('pub_cpu_mean_pct', src.get('pub_cpu_mean_pct', 'N/A')) if pub_data else src.get('pub_cpu_mean_pct', 'N/A')
    pub_cpu_peak = pub_data.get('pub_cpu_peak_pct', src.get('pub_cpu_peak_pct', 'N/A')) if pub_data else src.get('pub_cpu_peak_pct', 'N/A')

    cuda_sub_cpu_mean = cuda_data.get('sub_cpu_mean_pct', 'N/A') if cuda_data else 'N/A'
    cuda_sub_cpu_peak = cuda_data.get('sub_cpu_peak_pct', 'N/A') if cuda_data else 'N/A'
    fallback_sub_cpu_mean = fallback_data.get('sub_cpu_mean_pct', 'N/A') if fallback_data else 'N/A'
    fallback_sub_cpu_peak = fallback_data.get('sub_cpu_peak_pct', 'N/A') if fallback_data else 'N/A'

    pub_pss_mean = pub_data.get('pub_pss_mean_mb', src.get('pub_pss_mean_mb', 'N/A')) if pub_data else src.get('pub_pss_mean_mb', 'N/A')
    pub_pss_peak = pub_data.get('pub_pss_peak_mb', src.get('pub_pss_peak_mb', 'N/A')) if pub_data else src.get('pub_pss_peak_mb', 'N/A')
    cuda_sub_pss_mean = cuda_data.get('sub_pss_mean_mb', 'N/A') if cuda_data else 'N/A'
    cuda_sub_pss_peak = cuda_data.get('sub_pss_peak_mb', 'N/A') if cuda_data else 'N/A'
    fallback_sub_pss_mean = fallback_data.get('sub_pss_mean_mb', 'N/A') if fallback_data else 'N/A'
    fallback_sub_pss_peak = fallback_data.get('sub_pss_peak_mb', 'N/A') if fallback_data else 'N/A'

    gpu_util = src.get('gpu_util_pct', 'N/A')
    gpu_freq = src.get('gpu_freq_mhz', 'N/A')
    emc_util = src.get('emc_util_pct', 'N/A')
    emc_freq = src.get('emc_freq_mhz', 'N/A')

    merged_header = [
        'resolution', 'size_bytes', 'target_hz', 'actual_pub_fps',
        'cuda_recv_fps', 'fallback_recv_fps',
        'total_published_frames',
        'cuda_received_frames', 'fallback_received_frames',
        'cuda_dropped_frames', 'fallback_dropped_frames',
        'cuda_loss_pct', 'fallback_loss_pct',
        'cuda_cb_p50_ms', 'cuda_cb_p95_ms', 'cuda_cb_p99_ms', 'cuda_cb_mean_ms', 'cuda_cb_min_ms', 'cuda_cb_max_ms', 'cuda_cb_std_ms', 'cuda_cb_outliers',
        'cuda_br_p50_ms', 'fallback_p50_ms', 'p50_speedup',
        'cuda_br_p95_ms', 'fallback_p95_ms',
        'cuda_br_p99_ms', 'fallback_p99_ms', 'p99_speedup',
        'cuda_br_mean_ms', 'fallback_mean_ms', 'mean_speedup',
        'cuda_br_min_ms', 'fallback_min_ms',
        'cuda_br_max_ms', 'fallback_max_ms',
        'cuda_br_std_ms', 'fallback_std_ms',
        'cuda_br_outliers', 'fallback_outliers',
        'pub_cpu_mean_pct', 'pub_cpu_peak_pct',
        'cuda_sub_cpu_mean_pct', 'cuda_sub_cpu_peak_pct',
        'fallback_sub_cpu_mean_pct', 'fallback_sub_cpu_peak_pct',
        'pub_pss_mean_mb', 'pub_pss_peak_mb',
        'cuda_sub_pss_mean_mb', 'cuda_sub_pss_peak_mb',
        'fallback_sub_pss_mean_mb', 'fallback_sub_pss_peak_mb',
        'gpu_util_pct', 'gpu_freq_mhz', 'emc_util_pct', 'emc_freq_mhz',
    ]

    merged_row = [
        resolution, size_bytes, target_hz, str(actual_pub_fps),
        str(cuda_recv_fps), str(fallback_recv_fps),
        str(total_published),
        str(cuda_frames), str(fallback_frames),
        str(cuda_lost), str(fallback_lost),
        str(cuda_loss_pct), str(fallback_loss_pct),
        str(cuda_cb_p50), str(cuda_cb_p95), str(cuda_cb_p99), str(cuda_cb_mean), str(cuda_cb_min), str(cuda_cb_max), str(cuda_cb_std), str(cuda_cb_outliers),
        str(cuda_br_p50), str(fallback_p50), p50_speedup,
        str(cuda_br_p95), str(fallback_p95),
        str(cuda_br_p99), str(fallback_p99), p99_speedup,
        str(cuda_br_mean), str(fallback_mean), mean_speedup,
        str(cuda_br_min), str(fallback_min),
        str(cuda_br_max), str(fallback_max),
        str(cuda_br_std), str(fallback_std),
        str(cuda_br_outliers), str(fallback_outliers),
        str(pub_cpu_mean), str(pub_cpu_peak),
        str(cuda_sub_cpu_mean), str(cuda_sub_cpu_peak),
        str(fallback_sub_cpu_mean), str(fallback_sub_cpu_peak),
        str(pub_pss_mean), str(pub_pss_peak),
        str(cuda_sub_pss_mean), str(cuda_sub_pss_peak),
        str(fallback_sub_pss_mean), str(fallback_sub_pss_peak),
        str(gpu_util), str(gpu_freq), str(emc_util), str(emc_freq),
    ]

    file_exists = os.path.exists(csv_file) and os.path.getsize(csv_file) > 0
    needs_header = True
    if file_exists:
        try:
            with open(csv_file, 'r') as f:
                first_line = f.readline()
                if first_line.strip() == ','.join(merged_header):
                    needs_header = False
        except OSError:
            pass

    try:
        os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
        with open(csv_file, 'a' if file_exists and not needs_header else 'w') as f:
            if needs_header:
                f.write(','.join(merged_header) + '\n')
            f.write(','.join(merged_row) + '\n')
    except OSError as err:
        print(f'Error writing merged CSV to {csv_file}: {err}')

    # Output formatted comparison table
    cu_col = "CUDA (Zero-Copy)" if expect_cuda else "CUDA (Disabled)"
    fb_col = "Fallback (CPU / Wire)" if expect_fallback else "Fallback (Disabled)"

    print('\n' + '=' * 105)
    print(f' CUDA ZERO-COPY vs CPU FALLBACK BENCHMARK SUMMARY  [{resolution} @ {target_hz} Hz]')
    print('=' * 105)
    print(f" {'Metric':<28} | {cu_col:<22} | {fb_col:<22} | {'Speedup / Ratio'}")
    print('-' * 105)
    print(f" {'Actual Publishing Rate':<28} | {str(actual_pub_fps) + ' FPS':<22} | {str(actual_pub_fps) + ' FPS':<22} | Target: {target_hz} Hz")
    cu_recv_str = f'{cuda_recv_fps} FPS' if cuda_recv_fps != 'N/A' else 'N/A'
    fb_recv_str = f'{fallback_recv_fps} FPS' if fallback_recv_fps != 'N/A' else 'N/A'
    print(f" {'Actual Receiving Rate':<28} | {cu_recv_str:<22} | {fb_recv_str:<22} |")
    print(f" {'Total Published Frames':<28} | {str(total_published):<22} | {str(total_published):<22} |")
    print(f" {'Received Measured Frames':<28} | {str(cuda_frames):<22} | {str(fallback_frames):<22} |")
    cu_lost_str = f'{cuda_lost} ({cuda_loss_pct}%)' if cuda_lost != 'N/A' else 'N/A'
    fb_lost_str = f'{fallback_lost} ({fallback_loss_pct}%)' if fallback_lost != 'N/A' else 'N/A'
    print(f" {'Dropped Frames (Loss %)':<28} | {cu_lost_str:<22} | {fb_lost_str:<22} |")
    print('-' * 105)
    print(" [1] Subscriber Callback Entry Latency (Pub-to-Callback Entry):")
    cu_cb_percentiles = f'{cuda_cb_p50} / {cuda_cb_p95} / {cuda_cb_p99} ms' if cuda_cb_p50 != 'N/A' else 'N/A'
    fb_percentiles = f'{fallback_p50} / {fallback_p95} / {fallback_p99} ms' if fallback_p50 != 'N/A' else 'N/A'
    print(f" {'  P50 / P95 / P99':<28} | {cu_cb_percentiles:<22} | {fb_percentiles:<22} |")
    cu_cb_range = f'{cuda_cb_mean} [{cuda_cb_min}, {cuda_cb_max}]' if cuda_cb_mean != 'N/A' else 'N/A'
    fb_range = f'{fallback_mean} [{fallback_min}, {fallback_max}]' if fallback_mean != 'N/A' else 'N/A'
    print(f" {'  Mean [Min, Max] Latency':<28} | {cu_cb_range:<22} | {fb_range:<22} |")
    cu_cb_std_str = f'{cuda_cb_std} ms' if cuda_cb_std != 'N/A' else 'N/A'
    fb_std_str = f'{fallback_std} ms' if fallback_std != 'N/A' else 'N/A'
    print(f" {'  Std Dev (Jitter)':<28} | {cu_cb_std_str:<22} | {fb_std_str:<22} |")
    cu_cb_outlier_str = f'{cuda_cb_outliers} frames' if cuda_cb_outliers != 'N/A' else 'N/A'
    fb_outlier_str = f'{fallback_outliers} frames' if fallback_outliers != 'N/A' else 'N/A'
    print(f" {'  Outliers (> 3-sigma)':<28} | {cu_cb_outlier_str:<22} | {fb_outlier_str:<22} |")
    print('-' * 105)
    print(" [2] GPU-Ready Latency (Pub-to-Data Fully Available on Subscriber GPU):")
    cu_br_p50_str = f'{cuda_br_p50} ms' if cuda_br_p50 != 'N/A' else 'N/A'
    fb_p50_str = f'{fallback_p50} ms' if fallback_p50 != 'N/A' else 'N/A'
    print(f" {'  P50 Latency (Median)':<28} | {cu_br_p50_str:<22} | {fb_p50_str:<22} | {p50_speedup}")
    cu_br_p95_str = f'{cuda_br_p95} ms' if cuda_br_p95 != 'N/A' else 'N/A'
    fb_p95_str = f'{fallback_p95} ms' if fallback_p95 != 'N/A' else 'N/A'
    print(f" {'  P95 Latency':<28} | {cu_br_p95_str:<22} | {fb_p95_str:<22} |")
    cu_br_p99_str = f'{cuda_br_p99} ms' if cuda_br_p99 != 'N/A' else 'N/A'
    fb_p99_str = f'{fallback_p99} ms' if fallback_p99 != 'N/A' else 'N/A'
    print(f" {'  P99 Latency (Tail)':<28} | {cu_br_p99_str:<22} | {fb_p99_str:<22} | {p99_speedup}")
    cu_br_mean_str = f'{cuda_br_mean} ms' if cuda_br_mean != 'N/A' else 'N/A'
    fb_mean_str = f'{fallback_mean} ms' if fallback_mean != 'N/A' else 'N/A'
    print(f" {'  Mean Latency':<28} | {cu_br_mean_str:<22} | {fb_mean_str:<22} | {mean_speedup}")
    cu_br_min_str = f'{cuda_br_min} ms' if cuda_br_min != 'N/A' else 'N/A'
    fb_min_str = f'{fallback_min} ms' if fallback_min != 'N/A' else 'N/A'
    print(f" {'  Min Latency':<28} | {cu_br_min_str:<22} | {fb_min_str:<22} |")
    cu_br_max_str = f'{cuda_br_max} ms' if cuda_br_max != 'N/A' else 'N/A'
    fb_max_str = f'{fallback_max} ms' if fallback_max != 'N/A' else 'N/A'
    print(f" {'  Max Latency':<28} | {cu_br_max_str:<22} | {fb_max_str:<22} |")
    cu_br_std_str = f'{cuda_br_std} ms' if cuda_br_std != 'N/A' else 'N/A'
    print(f" {'  Std Dev (Jitter)':<28} | {cu_br_std_str:<22} | {fb_std_str:<22} |")
    cu_br_outliers_str = f'{cuda_br_outliers} frames' if cuda_br_outliers != 'N/A' else 'N/A'
    print(f" {'  Outliers (> 3-sigma)':<28} | {cu_br_outliers_str:<22} | {fb_outlier_str:<22} | (Never discarded)")
    cu_op_str = f'{cuda_device_op} ms' if cuda_device_op != 'N/A' else 'N/A'
    fb_op_str = f'{fallback_device_op} ms' if fallback_device_op != 'N/A' else 'N/A'
    print(f" {'  Device Op (H2D / Imp-Sync)':<28} | {cu_op_str:<22} | {fb_op_str:<22} | H2D copy vs CUDA import")
    print('-' * 105)
    print(" System Resources:")
    pub_cpu_str = f'Mean: {pub_cpu_mean}% | Peak: {pub_cpu_peak}%' if pub_cpu_mean != 'N/A' else 'N/A'
    print(f" {'  Publisher CPU':<28} | {pub_cpu_str}")
    cu_sub_cpu_str = f'Mean: {cuda_sub_cpu_mean}% (Peak {cuda_sub_cpu_peak}%)' if cuda_sub_cpu_mean != 'N/A' else 'N/A'
    fb_sub_cpu_str = f'Mean: {fallback_sub_cpu_mean}% (Peak {fallback_sub_cpu_peak}%)' if fallback_sub_cpu_mean != 'N/A' else 'N/A'
    print(f" {'  Subscriber CPU':<28} | CUDA: {cu_sub_cpu_str} | Fallback: {fb_sub_cpu_str}")
    pub_pss_str = f'Mean: {pub_pss_mean} MB | Peak: {pub_pss_peak} MB' if pub_pss_mean != 'N/A' else 'N/A'
    print(f" {'  Publisher PSS':<28} | {pub_pss_str}")
    cu_sub_pss_str = f'Mean: {cuda_sub_pss_mean} MB (Peak {cuda_sub_pss_peak} MB)' if cuda_sub_pss_mean != 'N/A' else 'N/A'
    fb_sub_pss_str = f'Mean: {fallback_sub_pss_mean} MB (Peak {fallback_sub_pss_peak} MB)' if fallback_sub_pss_mean != 'N/A' else 'N/A'
    print(f" {'  Subscriber PSS':<28} | CUDA: {cu_sub_pss_str} | Fallback: {fb_sub_pss_str}")
    if gpu_util != 'N/A':
        gpu_str = f'{gpu_util}%' + (f' @ {gpu_freq} MHz' if gpu_freq != 'N/A' else '')
        print(f" {'  GPU Util / Freq':<28} | {gpu_str}")
    if emc_util != 'N/A' or emc_freq != 'N/A':
        emc_str = f'{emc_util}%' + (f' @ {emc_freq} MHz' if emc_freq != 'N/A' else '')
        print(f" {'  EMC Memory Controller':<28} | {emc_str}")
    print('=' * 105)
    print(f' Results saved to: {csv_file}\n')


def get_rmw_short_name():
    rmw = os.environ.get('RMW_IMPLEMENTATION', '').strip()
    if not rmw:
        try:
            import rclpy
            rmw = rclpy.get_rmw_implementation_identifier()
        except Exception:
            rmw = ''
    rmw_map = {
        'rmw_fastrtps_cpp': 'fastrtps',
        'rmw_zenoh_cpp': 'zenoh',
    }
    if rmw not in rmw_map:
        print(
            f"ERROR: Unsupported RMW_IMPLEMENTATION '{rmw or '<unknown>'}'. "
            "Only rmw_fastrtps_cpp and rmw_zenoh_cpp are supported.",
            file=sys.stderr,
        )
        sys.exit(1)
    return rmw_map[rmw]


def generate_launch_description():
    rmw_tag = get_rmw_short_name()
    width_arg = DeclareLaunchArgument(
        'width', default_value='1920',
        description='Image width in pixels')
    height_arg = DeclareLaunchArgument(
        'height', default_value='1080',
        description='Image height in pixels')
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='30.0',
        description='Publishing rate in Hz')
    max_msgs_arg = DeclareLaunchArgument(
        'max_msgs', default_value='1000',
        description='Number of messages to publish (0 for continuous)')
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value=f'/tmp/cuda_fallback_comparison_{rmw_tag}.csv',
        description='Path to save output comparison CSV results')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    cuda_device_id_arg = DeclareLaunchArgument(
        'cuda_device_id', default_value='0',
        description='CUDA GPU device ordinal')
    sub_mode_arg = DeclareLaunchArgument(
        'sub_mode', default_value='both',
        description="Subscriber mode: 'both' (default: runs both CUDA and CPU subscribers), 'cuda' (only CUDA subscriber), or 'cpu' (only CPU subscriber)")
    cuda_latency_csv_path_arg = DeclareLaunchArgument(
        'cuda_latency_csv_path', default_value='',
        description='Optional path to save raw per-frame CUDA latency CSV')
    fallback_latency_csv_path_arg = DeclareLaunchArgument(
        'fallback_latency_csv_path', default_value='',
        description='Optional path to save raw per-frame fallback CPU latency CSV')

    effective_max_msgs = PythonExpression([
        "'1000' if int('", LaunchConfiguration('max_msgs'), "') <= 0 else '",
        LaunchConfiguration('max_msgs'), "'"
    ])


    run_cuda_sub = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['both', 'cuda', 'cu', 'gpu', 'all']"
        ])
    )

    run_fallback_sub = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['both', 'cpu', 'fallback', 'wire', 'all']"
        ])
    )

    cuda_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".cuda.tmp'"])
    fallback_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".fallback.tmp'"])
    pub_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".pub.tmp'"])

    cuda_subscriber_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='cuda_subscriber',
        condition=run_cuda_sub,
        output='screen',
        parameters=[{
            'topic': 'test_cuda_image',
            'result_prefix': 'cuda_sub',
            'use_cuda': True,
            'require_cuda': True,
            'verify_content': True,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': cuda_tmp_csv,
            'latency_csv_path': LaunchConfiguration('cuda_latency_csv_path'),
            'auto_exit': True,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    fallback_subscriber_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='fallback_subscriber',
        condition=run_fallback_sub,
        output='screen',
        parameters=[{
            'topic': 'test_cuda_image',
            'result_prefix': 'fallback_sub',
            'use_cuda': True,
            'require_cuda': False,
            'verify_content': True,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': fallback_tmp_csv,
            'latency_csv_path': LaunchConfiguration('fallback_latency_csv_path'),
            'auto_exit': True,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    publisher_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_publisher_node',
        name='cuda_publisher',
        output='screen',
        parameters=[{
            'topic': 'test_cuda_image',
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_cuda': True,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'summary_csv_path': pub_tmp_csv,
            'auto_exit': True,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    delayed_publisher = TimerAction(
        period=1.0,
        actions=[publisher_node]
    )

    exit_cuda_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cuda_subscriber_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='CUDA subscriber finished benchmark targets')),
            ]
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['cuda', 'cu', 'gpu']"
        ]))
    )

    exit_fallback_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fallback_subscriber_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Subscriber finished benchmark targets')),
            ]
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['both', 'cpu', 'fallback', 'wire', 'all']"
        ]))
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=merge_and_print_csv_results)
            ]
        )
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        rate_arg,
        max_msgs_arg,
        warmup_msgs_arg,
        csv_path_arg,
        history_depth_arg,
        cuda_device_id_arg,
        sub_mode_arg,
        cuda_latency_csv_path_arg,
        fallback_latency_csv_path_arg,
        OpaqueFunction(function=init_csv_file),
        cuda_subscriber_node,
        fallback_subscriber_node,
        delayed_publisher,
        exit_cuda_handler,
        exit_fallback_handler,
        shutdown_event_handler,
    ])
