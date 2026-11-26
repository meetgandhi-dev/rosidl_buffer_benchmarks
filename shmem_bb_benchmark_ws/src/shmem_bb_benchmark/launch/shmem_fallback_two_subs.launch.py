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
            for ext in ['.shmem.tmp', '.fallback.tmp', '.pub.tmp']:
                tmp_path = f'{csv_file}{ext}'
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
        except OSError:
            pass


def merge_and_print_csv_results(*args, **kwargs):
    """Merge shmem and fallback results into a single comparison CSV and print summary."""
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

    shmem_tmp = f'{csv_file}.shmem.tmp'
    fallback_tmp = f'{csv_file}.fallback.tmp'
    pub_tmp = f'{csv_file}.pub.tmp'

    sub_mode = context.launch_configurations.get('sub_mode', 'both').strip().lower()
    expect_shmem = sub_mode in ['both', 'shmem', 'shm', 'all']
    expect_fallback = sub_mode in ['both', 'cpu', 'fallback', 'wire', 'all']

    # Wait briefly for active nodes to finalize writing summary files
    deadline = time.time() + 2.5
    while time.time() < deadline:
        shmem_ready = (not expect_shmem) or os.path.exists(shmem_tmp)
        fallback_ready = (not expect_fallback) or os.path.exists(fallback_tmp)
        if shmem_ready and fallback_ready:
            time.sleep(0.05)
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

    pub_data = read_summary(pub_tmp)
    shmem_data = read_summary(shmem_tmp) if expect_shmem else None
    fallback_data = read_summary(fallback_tmp) if expect_fallback else None

    # Clean up temporary files
    for tmp in [pub_tmp, shmem_tmp, fallback_tmp]:
        if os.path.exists(tmp):
            try:
                os.remove(tmp)
            except OSError:
                pass

    if not shmem_data and not fallback_data:
        return

    src = shmem_data or fallback_data
    width = context.launch_configurations.get('width', '')
    height = context.launch_configurations.get('height', '')
    res_fallback = f'{width}x{height}' if (width and height) else 'unknown'
    resolution = src.get('resolution', res_fallback)
    size_bytes = src.get('size_bytes', '')
    target_hz = src.get('target_hz', context.launch_configurations.get('rate', ''))
    frame_cap = context.launch_configurations.get('max_msgs', '')

    actual_pub_fps = pub_data.get('actual_pub_fps', src.get('actual_pub_fps', src.get('actual_pub_hz', target_hz))) if pub_data else src.get('actual_pub_fps', src.get('actual_pub_hz', target_hz))
    total_published = pub_data.get('total_published_frames', src.get('total_published_frames', src.get('total_sent', frame_cap))) if pub_data else src.get('total_published_frames', src.get('total_sent', frame_cap))

    shmem_recv_fps = shmem_data.get('actual_recv_fps', shmem_data.get('actual_recv_hz', '0.0')) if shmem_data else 'N/A'
    fallback_recv_fps = fallback_data.get('actual_recv_fps', fallback_data.get('actual_recv_hz', '0.0')) if fallback_data else 'N/A'

    shmem_frames = shmem_data.get('total_received_frames', shmem_data.get('received_frames', '0')) if shmem_data else 'N/A'
    fallback_frames = fallback_data.get('total_received_frames', fallback_data.get('received_frames', '0')) if fallback_data else 'N/A'

    shmem_lost = shmem_data.get('dropped_frames', shmem_data.get('lost_frames', '0')) if shmem_data else 'N/A'
    fallback_lost = fallback_data.get('dropped_frames', fallback_data.get('lost_frames', '0')) if fallback_data else 'N/A'

    shmem_loss_pct = shmem_data.get('frame_loss_pct', shmem_data.get('loss_pct', '0.0')) if shmem_data else 'N/A'
    fallback_loss_pct = fallback_data.get('frame_loss_pct', fallback_data.get('loss_pct', '0.0')) if fallback_data else 'N/A'

    shmem_p50 = shmem_data.get('p50_latency_ms', '0.0') if shmem_data else 'N/A'
    fallback_p50 = fallback_data.get('p50_latency_ms', '0.0') if fallback_data else 'N/A'

    shmem_p95 = shmem_data.get('p95_latency_ms', '0.0') if shmem_data else 'N/A'
    fallback_p95 = fallback_data.get('p95_latency_ms', '0.0') if fallback_data else 'N/A'

    shmem_p99 = shmem_data.get('p99_latency_ms', '0.0') if shmem_data else 'N/A'
    fallback_p99 = fallback_data.get('p99_latency_ms', '0.0') if fallback_data else 'N/A'

    shmem_mean = shmem_data.get('mean_latency_ms', shmem_data.get('avg_latency_ms', '0.0')) if shmem_data else 'N/A'
    fallback_mean = fallback_data.get('mean_latency_ms', fallback_data.get('avg_latency_ms', '0.0')) if fallback_data else 'N/A'

    shmem_min = shmem_data.get('min_latency_ms', '0.0') if shmem_data else 'N/A'
    fallback_min = fallback_data.get('min_latency_ms', '0.0') if fallback_data else 'N/A'

    shmem_max = shmem_data.get('max_latency_ms', '0.0') if shmem_data else 'N/A'
    fallback_max = fallback_data.get('max_latency_ms', '0.0') if fallback_data else 'N/A'

    shmem_std = shmem_data.get('std_dev_ms', '0.0') if shmem_data else 'N/A'
    fallback_std = fallback_data.get('std_dev_ms', '0.0') if fallback_data else 'N/A'

    shmem_outliers = shmem_data.get('outliers_3sigma_count', '0') if shmem_data else 'N/A'
    fallback_outliers = fallback_data.get('outliers_3sigma_count', '0') if fallback_data else 'N/A'

    shmem_dr_p50 = shmem_data.get('data_ready_p50_ms', 'N/A') if shmem_data else 'N/A'
    fallback_dr_p50 = fallback_data.get('data_ready_p50_ms', 'N/A') if fallback_data else 'N/A'
    shmem_dr_p95 = shmem_data.get('data_ready_p95_ms', 'N/A') if shmem_data else 'N/A'
    fallback_dr_p95 = fallback_data.get('data_ready_p95_ms', 'N/A') if fallback_data else 'N/A'
    shmem_dr_p99 = shmem_data.get('data_ready_p99_ms', 'N/A') if shmem_data else 'N/A'
    fallback_dr_p99 = fallback_data.get('data_ready_p99_ms', 'N/A') if fallback_data else 'N/A'
    shmem_dr_mean = shmem_data.get('data_ready_mean_ms', 'N/A') if shmem_data else 'N/A'
    fallback_dr_mean = fallback_data.get('data_ready_mean_ms', 'N/A') if fallback_data else 'N/A'
    shmem_dr_std = shmem_data.get('data_ready_std_dev_ms', 'N/A') if shmem_data else 'N/A'
    fallback_dr_std = fallback_data.get('data_ready_std_dev_ms', 'N/A') if fallback_data else 'N/A'

    pub_cpu_mean = pub_data.get('pub_cpu_mean_pct', src.get('pub_cpu_mean_pct', 'N/A')) if pub_data else src.get('pub_cpu_mean_pct', 'N/A')
    pub_cpu_peak = pub_data.get('pub_cpu_peak_pct', src.get('pub_cpu_peak_pct', 'N/A')) if pub_data else src.get('pub_cpu_peak_pct', 'N/A')

    shmem_sub_cpu_mean = shmem_data.get('sub_cpu_mean_pct', 'N/A') if shmem_data else 'N/A'
    shmem_sub_cpu_peak = shmem_data.get('sub_cpu_peak_pct', 'N/A') if shmem_data else 'N/A'
    fallback_sub_cpu_mean = fallback_data.get('sub_cpu_mean_pct', 'N/A') if fallback_data else 'N/A'
    fallback_sub_cpu_peak = fallback_data.get('sub_cpu_peak_pct', 'N/A') if fallback_data else 'N/A'

    pub_pss_mean = pub_data.get('pub_pss_mean_mb', src.get('pub_pss_mean_mb', 'N/A')) if pub_data else src.get('pub_pss_mean_mb', 'N/A')
    pub_pss_peak = pub_data.get('pub_pss_peak_mb', src.get('pub_pss_peak_mb', 'N/A')) if pub_data else src.get('pub_pss_peak_mb', 'N/A')
    shmem_sub_pss_mean = shmem_data.get('sub_pss_mean_mb', 'N/A') if shmem_data else 'N/A'
    shmem_sub_pss_peak = shmem_data.get('sub_pss_peak_mb', 'N/A') if shmem_data else 'N/A'
    fallback_sub_pss_mean = fallback_data.get('sub_pss_mean_mb', 'N/A') if fallback_data else 'N/A'
    fallback_sub_pss_peak = fallback_data.get('sub_pss_peak_mb', 'N/A') if fallback_data else 'N/A'

    def calc_speedup(fb_str, shm_str):
        try:
            fb = float(fb_str)
            shm = float(shm_str)
            if shm > 0.0:
                return f'{fb / shm:.2f}x'
        except (ValueError, TypeError):
            pass
        return 'N/A'

    p50_speedup = calc_speedup(fallback_p50, shmem_p50)
    p99_speedup = calc_speedup(fallback_p99, shmem_p99)
    mean_speedup = calc_speedup(fallback_mean, shmem_mean)

    merged_header = [
        'resolution', 'size_bytes', 'target_hz', 'actual_pub_fps',
        'shmem_recv_fps', 'fallback_recv_fps',
        'total_published_frames',
        'shmem_received_frames', 'fallback_received_frames',
        'shmem_dropped_frames', 'fallback_dropped_frames',
        'shmem_loss_pct', 'fallback_loss_pct',
        'shmem_p50_ms', 'fallback_p50_ms', 'p50_speedup',
        'shmem_p95_ms', 'fallback_p95_ms',
        'shmem_p99_ms', 'fallback_p99_ms', 'p99_speedup',
        'shmem_mean_ms', 'fallback_mean_ms', 'mean_speedup',
        'shmem_min_ms', 'fallback_min_ms',
        'shmem_max_ms', 'fallback_max_ms',
        'shmem_std_ms', 'fallback_std_ms',
        'shmem_outliers', 'fallback_outliers',
        'shmem_dr_p50_ms', 'fallback_dr_p50_ms',
        'shmem_dr_p95_ms', 'fallback_dr_p95_ms',
        'shmem_dr_p99_ms', 'fallback_dr_p99_ms',
        'shmem_dr_mean_ms', 'fallback_dr_mean_ms',
        'shmem_dr_std_ms', 'fallback_dr_std_ms',
        'pub_cpu_mean_pct', 'pub_cpu_peak_pct',
        'shmem_sub_cpu_mean_pct', 'shmem_sub_cpu_peak_pct',
        'fallback_sub_cpu_mean_pct', 'fallback_sub_cpu_peak_pct',
        'pub_pss_mean_mb', 'pub_pss_peak_mb',
        'shmem_sub_pss_mean_mb', 'shmem_sub_pss_peak_mb',
        'fallback_sub_pss_mean_mb', 'fallback_sub_pss_peak_mb',
    ]

    merged_row = [
        resolution, size_bytes, target_hz, str(actual_pub_fps),
        str(shmem_recv_fps), str(fallback_recv_fps),
        str(total_published),
        str(shmem_frames), str(fallback_frames),
        str(shmem_lost), str(fallback_lost),
        str(shmem_loss_pct), str(fallback_loss_pct),
        str(shmem_p50), str(fallback_p50), p50_speedup,
        str(shmem_p95), str(fallback_p95),
        str(shmem_p99), str(fallback_p99), p99_speedup,
        str(shmem_mean), str(fallback_mean), mean_speedup,
        str(shmem_min), str(fallback_min),
        str(shmem_max), str(fallback_max),
        str(shmem_std), str(fallback_std),
        str(shmem_outliers), str(fallback_outliers),
        str(shmem_dr_p50), str(fallback_dr_p50),
        str(shmem_dr_p95), str(fallback_dr_p95),
        str(shmem_dr_p99), str(fallback_dr_p99),
        str(shmem_dr_mean), str(fallback_dr_mean),
        str(shmem_dr_std), str(fallback_dr_std),
        str(pub_cpu_mean), str(pub_cpu_peak),
        str(shmem_sub_cpu_mean), str(shmem_sub_cpu_peak),
        str(fallback_sub_cpu_mean), str(fallback_sub_cpu_peak),
        str(pub_pss_mean), str(pub_pss_peak),
        str(shmem_sub_pss_mean), str(shmem_sub_pss_peak),
        str(fallback_sub_pss_mean), str(fallback_sub_pss_peak),
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
    shm_col = "SHMEM (Zero-Copy)" if expect_shmem else "SHMEM (Disabled)"
    fb_col = "Fallback (CPU / Wire)" if expect_fallback else "Fallback (Disabled)"

    print('\n' + '=' * 105)
    print(f' ZERO-COPY SHMEM vs CPU FALLBACK BENCHMARK SUMMARY  [{resolution} @ {target_hz} Hz]')
    print('=' * 105)
    print(f" {'Metric':<28} | {shm_col:<22} | {fb_col:<22} | {'Speedup / Ratio'}")
    print('-' * 105)
    print(f" {'Actual Publishing Rate':<28} | {str(actual_pub_fps) + ' FPS':<22} | {str(actual_pub_fps) + ' FPS':<22} | Target: {target_hz} Hz")
    shm_recv_str = f'{shmem_recv_fps} FPS' if shmem_recv_fps != 'N/A' else 'N/A'
    fb_recv_str = f'{fallback_recv_fps} FPS' if fallback_recv_fps != 'N/A' else 'N/A'
    print(f" {'Actual Receiving Rate':<28} | {shm_recv_str:<22} | {fb_recv_str:<22} |")
    print(f" {'Total Published Frames':<28} | {str(total_published):<22} | {str(total_published):<22} |")
    print(f" {'Total Received Frames':<28} | {str(shmem_frames):<22} | {str(fallback_frames):<22} |")
    shm_drop_str = f'{shmem_lost} ({shmem_loss_pct}%)' if shmem_lost != 'N/A' else 'N/A'
    fb_drop_str = f'{fallback_lost} ({fallback_loss_pct}%)' if fallback_lost != 'N/A' else 'N/A'
    print(f" {'Dropped Frames (Loss %)':<28} | {shm_drop_str:<22} | {fb_drop_str:<22} |")
    print('-' * 105)
    print(" [1] Callback-Entry Latency (Pub-to-Callback Entry):")
    shm_p50_str = f'{shmem_p50} ms' if shmem_p50 != 'N/A' else 'N/A'
    fb_p50_str = f'{fallback_p50} ms' if fallback_p50 != 'N/A' else 'N/A'
    print(f" {'  P50 Latency (Median)':<28} | {shm_p50_str:<22} | {fb_p50_str:<22} | {p50_speedup}")
    shm_p95_str = f'{shmem_p95} ms' if shmem_p95 != 'N/A' else 'N/A'
    fb_p95_str = f'{fallback_p95} ms' if fallback_p95 != 'N/A' else 'N/A'
    print(f" {'  P95 Latency':<28} | {shm_p95_str:<22} | {fb_p95_str:<22} |")
    shm_p99_str = f'{shmem_p99} ms' if shmem_p99 != 'N/A' else 'N/A'
    fb_p99_str = f'{fallback_p99} ms' if fallback_p99 != 'N/A' else 'N/A'
    print(f" {'  P99 Latency (Tail)':<28} | {shm_p99_str:<22} | {fb_p99_str:<22} | {p99_speedup}")
    shm_mean_str = f'{shmem_mean} ms' if shmem_mean != 'N/A' else 'N/A'
    fb_mean_str = f'{fallback_mean} ms' if fallback_mean != 'N/A' else 'N/A'
    print(f" {'  Mean Latency':<28} | {shm_mean_str:<22} | {fb_mean_str:<22} | {mean_speedup}")
    shm_min_str = f'{shmem_min} ms' if shmem_min != 'N/A' else 'N/A'
    fb_min_str = f'{fallback_min} ms' if fallback_min != 'N/A' else 'N/A'
    print(f" {'  Min Latency':<28} | {shm_min_str:<22} | {fb_min_str:<22} |")
    shm_max_str = f'{shmem_max} ms' if shmem_max != 'N/A' else 'N/A'
    fb_max_str = f'{fallback_max} ms' if fallback_max != 'N/A' else 'N/A'
    print(f" {'  Max Latency':<28} | {shm_max_str:<22} | {fb_max_str:<22} |")
    shm_std_str = f'{shmem_std} ms' if shmem_std != 'N/A' else 'N/A'
    fb_std_str = f'{fallback_std} ms' if fallback_std != 'N/A' else 'N/A'
    print(f" {'  Std Dev (Jitter)':<28} | {shm_std_str:<22} | {fb_std_str:<22} |")
    print(f" {'  Outliers (> 3-sigma)':<28} | {str(shmem_outliers):<22} | {str(fallback_outliers):<22} |")
    print('-' * 105)
    print(" [2] Backend Data-Ready Latency (Pub-to-Payload Pointer Accessible):")
    shm_dr_p50_str = f'{shmem_dr_p50} ms' if shmem_dr_p50 != 'N/A' else 'N/A'
    fb_dr_p50_str = f'{fallback_dr_p50} ms' if fallback_dr_p50 != 'N/A' else 'N/A'
    print(f" {'  P50 Data-Ready Latency':<28} | {shm_dr_p50_str:<22} | {fb_dr_p50_str:<22} |")
    shm_dr_p95_str = f'{shmem_dr_p95} ms' if shmem_dr_p95 != 'N/A' else 'N/A'
    fb_dr_p95_str = f'{fallback_dr_p95} ms' if fallback_dr_p95 != 'N/A' else 'N/A'
    print(f" {'  P95 Data-Ready Latency':<28} | {shm_dr_p95_str:<22} | {fb_dr_p95_str:<22} |")
    shm_dr_p99_str = f'{shmem_dr_p99} ms' if shmem_dr_p99 != 'N/A' else 'N/A'
    fb_dr_p99_str = f'{fallback_dr_p99} ms' if fallback_dr_p99 != 'N/A' else 'N/A'
    print(f" {'  P99 Data-Ready Latency':<28} | {shm_dr_p99_str:<22} | {fb_dr_p99_str:<22} |")
    shm_dr_mean_str = f'{shmem_dr_mean} ms' if shmem_dr_mean != 'N/A' else 'N/A'
    fb_dr_mean_str = f'{fallback_dr_mean} ms' if fallback_dr_mean != 'N/A' else 'N/A'
    print(f" {'  Mean Data-Ready Latency':<28} | {shm_dr_mean_str:<22} | {fb_dr_mean_str:<22} |")
    shm_dr_std_str = f'{shmem_dr_std} ms' if shmem_dr_std != 'N/A' else 'N/A'
    fb_dr_std_str = f'{fallback_dr_std} ms' if fallback_dr_std != 'N/A' else 'N/A'
    print(f" {'  Std Dev Data-Ready':<28} | {shm_dr_std_str:<22} | {fb_dr_std_str:<22} |")
    print('-' * 105)
    print(" System Resources:")
    pub_cpu_str = f'{pub_cpu_mean}% (Peak {pub_cpu_peak}%)' if pub_cpu_mean != 'N/A' else 'N/A'
    print(f" {'  Publisher CPU':<28} | {pub_cpu_str:<22} | {pub_cpu_str:<22} | (Process CPU)")
    shm_cpu_str = f'{shmem_sub_cpu_mean}% (Peak {shmem_sub_cpu_peak}%)' if shmem_sub_cpu_mean != 'N/A' else 'N/A'
    fb_cpu_str = f'{fallback_sub_cpu_mean}% (Peak {fallback_sub_cpu_peak}%)' if fallback_sub_cpu_mean != 'N/A' else 'N/A'
    print(f" {'  Subscriber CPU':<28} | {shm_cpu_str:<22} | {fb_cpu_str:<22} | (Process CPU)")
    pub_pss_str = f'{pub_pss_mean} MB (Peak {pub_pss_peak} MB)' if pub_pss_mean != 'N/A' else 'N/A'
    print(f" {'  Publisher PSS':<28} | {pub_pss_str:<22} | {pub_pss_str:<22} | (/proc/self/smaps_rollup)")
    shm_pss_str = f'{shmem_sub_pss_mean} MB (Peak {shmem_sub_pss_peak} MB)' if shmem_sub_pss_mean != 'N/A' else 'N/A'
    fb_pss_str = f'{fallback_sub_pss_mean} MB (Peak {fallback_sub_pss_peak} MB)' if fallback_sub_pss_mean != 'N/A' else 'N/A'
    print(f" {'  Subscriber PSS':<28} | {shm_pss_str:<22} | {fb_pss_str:<22} | (/proc/self/smaps_rollup)")
    print('=' * 105)
    print(f' Results saved to: {csv_file}\n')


def generate_launch_description():
    # Primary arguments: width, height, rate, and max_msgs
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
        description='Maximum messages to publish/receive (default: 1000; set 0 for continuous)')
    max_messages_alias_arg = DeclareLaunchArgument(
        'max_messages', default_value='',
        description='Alias for max_msgs')
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')

    # Output and runtime configuration
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/shmem_fallback_comparison.csv',
        description='Path to save output comparison CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='test_shmem_image',
        description='Topic name for image streaming')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    auto_exit_arg = DeclareLaunchArgument(
        'auto_exit', default_value='',
        description='Whether nodes should automatically exit (default: true when max_msgs > 0)')
    verify_content_arg = DeclareLaunchArgument(
        'verify_content', default_value='true',
        description='Whether subscribers verify pixel payload byte patterns')
    sub_mode_arg = DeclareLaunchArgument(
        'sub_mode', default_value='both',
        description="Subscriber mode: 'both' (default: runs both SHMEM and CPU subscribers), 'shmem' (only SHMEM subscriber), or 'cpu' (only CPU subscriber)")
    shmem_latency_csv_path_arg = DeclareLaunchArgument(
        'shmem_latency_csv_path', default_value='',
        description='Optional path to save raw per-frame SHMEM latency CSV')
    fallback_latency_csv_path_arg = DeclareLaunchArgument(
        'fallback_latency_csv_path', default_value='',
        description='Optional path to save raw per-frame fallback CPU latency CSV')

    # Resolve effective max messages
    effective_max_msgs = PythonExpression([
        "'", LaunchConfiguration('max_messages'), "' if '",
        LaunchConfiguration('max_messages'), "' != '' else '",
        LaunchConfiguration('max_msgs'), "'"
    ])

    # Auto exit expression: true if max_messages > 0
    auto_exit_expr = PythonExpression([
        "'", LaunchConfiguration('auto_exit'), "' if '",
        LaunchConfiguration('auto_exit'), "' != '' else (",
        "'true' if int('", effective_max_msgs, "') > 0 else 'false')"
    ])

    verify_content_expr = PythonExpression([
        "'", LaunchConfiguration('verify_content'), "'.strip().lower() in ['true', '1', 'yes']"
    ])

    run_shmem_sub = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['both', 'shmem', 'shm', 'all']"
        ])
    )

    run_fallback_sub = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['both', 'cpu', 'fallback', 'wire', 'all']"
        ])
    )

    # Temporary CSV paths for the two individual subscriber nodes and publisher node
    shmem_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".shmem.tmp'"])
    fallback_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".fallback.tmp'"])
    pub_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".pub.tmp'"])

    # 1. Subscriber 1: Zero-Copy Shared Memory Subscriber
    subscriber_shared = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='subscriber_shared',
        condition=run_shmem_sub,
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'sub_shared',
            'use_shm': True,
            'require_shmem': True,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': shmem_tmp_csv,
            'latency_csv_path': LaunchConfiguration('shmem_latency_csv_path'),
            'auto_exit': auto_exit_expr,
        }],
    )

    # 2. Subscriber 2: Fallback Wire (CPU copy) Subscriber
    subscriber_fallback = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='subscriber_fallback',
        condition=run_fallback_sub,
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'sub_fallback',
            'use_shm': False,
            'require_shmem': False,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': fallback_tmp_csv,
            'latency_csv_path': LaunchConfiguration('fallback_latency_csv_path'),
            'auto_exit': auto_exit_expr,
        }],
    )

    # 3. Publisher: Shared Memory Publisher (always runs in shmem mode)
    publisher = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_publisher_node',
        name='publisher_shared',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': True,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'summary_csv_path': pub_tmp_csv,
            'auto_exit': auto_exit_expr,
        }],
    )

    delayed_publisher = TimerAction(
        period=1.0,
        actions=[publisher]
    )

    exit_shmem_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=subscriber_shared,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Shared memory subscriber finished benchmark targets')),
            ]
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('sub_mode'), "'.strip().lower() in ['shmem', 'shm']"
        ]))
    )

    exit_fallback_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=subscriber_fallback,
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
        max_messages_alias_arg,
        warmup_msgs_arg,
        csv_path_arg,
        topic_arg,
        history_depth_arg,
        auto_exit_arg,
        verify_content_arg,
        sub_mode_arg,
        shmem_latency_csv_path_arg,
        fallback_latency_csv_path_arg,
        OpaqueFunction(function=init_csv_file),
        subscriber_shared,
        subscriber_fallback,
        delayed_publisher,
        exit_shmem_handler,
        exit_fallback_handler,
        shutdown_event_handler,
    ])
