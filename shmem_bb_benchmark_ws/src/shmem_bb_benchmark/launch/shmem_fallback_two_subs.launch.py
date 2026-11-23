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
            for ext in ['.shmem.tmp', '.fallback.tmp']:
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

    # Wait briefly for subscriber nodes to finalize writing summary files
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if os.path.exists(shmem_tmp) and os.path.exists(fallback_tmp):
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

    shmem_data = read_summary(shmem_tmp)
    fallback_data = read_summary(fallback_tmp)

    # Clean up temporary files
    for tmp in [shmem_tmp, fallback_tmp]:
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
    frame_cap = src.get('frame_cap', context.launch_configurations.get('max_msgs', ''))

    actual_pub_hz = shmem_data.get('actual_pub_hz', src.get('actual_pub_hz', target_hz)) if shmem_data else '0.0'
    shmem_recv_hz = shmem_data.get('actual_recv_hz', '0.0') if shmem_data else '0.0'
    fallback_recv_hz = fallback_data.get('actual_recv_hz', '0.0') if fallback_data else '0.0'

    shmem_frames = shmem_data.get('received_frames', '0') if shmem_data else '0'
    fallback_frames = fallback_data.get('received_frames', '0') if fallback_data else '0'

    shmem_lost = shmem_data.get('lost_frames', '0') if shmem_data else '0'
    fallback_lost = fallback_data.get('lost_frames', '0') if fallback_data else '0'

    shmem_loss_pct = shmem_data.get('loss_pct', '0.0') if shmem_data else '0.0'
    fallback_loss_pct = fallback_data.get('loss_pct', '0.0') if fallback_data else '0.0'

    shmem_p50 = shmem_data.get('p50_latency_ms', '0.0') if shmem_data else '0.0'
    fallback_p50 = fallback_data.get('p50_latency_ms', '0.0') if fallback_data else '0.0'

    shmem_p95 = shmem_data.get('p95_latency_ms', '0.0') if shmem_data else '0.0'
    fallback_p95 = fallback_data.get('p95_latency_ms', '0.0') if fallback_data else '0.0'

    shmem_p99 = shmem_data.get('p99_latency_ms', '0.0') if shmem_data else '0.0'
    fallback_p99 = fallback_data.get('p99_latency_ms', '0.0') if fallback_data else '0.0'

    shmem_mean = shmem_data.get('mean_latency_ms', shmem_data.get('avg_latency_ms', '0.0')) if shmem_data else '0.0'
    fallback_mean = fallback_data.get('mean_latency_ms', fallback_data.get('avg_latency_ms', '0.0')) if fallback_data else '0.0'

    shmem_min = shmem_data.get('min_latency_ms', '0.0') if shmem_data else '0.0'
    fallback_min = fallback_data.get('min_latency_ms', '0.0') if fallback_data else '0.0'

    shmem_max = shmem_data.get('max_latency_ms', '0.0') if shmem_data else '0.0'
    fallback_max = fallback_data.get('max_latency_ms', '0.0') if fallback_data else '0.0'

    shmem_std = shmem_data.get('std_dev_ms', '0.0') if shmem_data else '0.0'
    fallback_std = fallback_data.get('std_dev_ms', '0.0') if fallback_data else '0.0'

    shmem_outliers = shmem_data.get('outliers_3sigma_count', '0') if shmem_data else '0'
    fallback_outliers = fallback_data.get('outliers_3sigma_count', '0') if fallback_data else '0'

    shmem_cpu_mean = shmem_data.get('cpu_util_mean_pct', 'N/A') if shmem_data else 'N/A'
    shmem_cpu_max = shmem_data.get('cpu_util_max_pct', 'N/A') if shmem_data else 'N/A'
    fallback_cpu_mean = fallback_data.get('cpu_util_mean_pct', 'N/A') if fallback_data else 'N/A'
    fallback_cpu_max = fallback_data.get('cpu_util_max_pct', 'N/A') if fallback_data else 'N/A'

    shmem_rss_mean = shmem_data.get('process_rss_mean_mb', 'N/A') if shmem_data else 'N/A'
    shmem_rss_peak = shmem_data.get('process_rss_peak_mb', 'N/A') if shmem_data else 'N/A'
    fallback_rss_mean = fallback_data.get('process_rss_mean_mb', 'N/A') if fallback_data else 'N/A'
    fallback_rss_peak = fallback_data.get('process_rss_peak_mb', 'N/A') if fallback_data else 'N/A'

    shmem_clock_mean = shmem_data.get('cpu_clock_mean_mhz', 'N/A') if shmem_data else 'N/A'
    shmem_clock_max = shmem_data.get('cpu_clock_max_mhz', 'N/A') if shmem_data else 'N/A'
    fallback_clock_mean = fallback_data.get('cpu_clock_mean_mhz', 'N/A') if fallback_data else 'N/A'
    fallback_clock_max = fallback_data.get('cpu_clock_max_mhz', 'N/A') if fallback_data else 'N/A'

    shmem_power_mean = shmem_data.get('power_mean_w', 'N/A') if shmem_data else 'N/A'
    shmem_power_max = shmem_data.get('power_max_w', 'N/A') if shmem_data else 'N/A'
    fallback_power_mean = fallback_data.get('power_mean_w', 'N/A') if fallback_data else 'N/A'
    fallback_power_max = fallback_data.get('power_max_w', 'N/A') if fallback_data else 'N/A'

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
        'resolution', 'size_bytes', 'target_hz', 'actual_pub_hz',
        'shmem_recv_hz', 'fallback_recv_hz',
        'shmem_frames', 'fallback_frames',
        'shmem_loss_pct', 'fallback_loss_pct',
        'shmem_p50_ms', 'fallback_p50_ms', 'p50_speedup',
        'shmem_p95_ms', 'fallback_p95_ms',
        'shmem_p99_ms', 'fallback_p99_ms', 'p99_speedup',
        'shmem_mean_ms', 'fallback_mean_ms', 'mean_speedup',
        'shmem_min_ms', 'fallback_min_ms',
        'shmem_max_ms', 'fallback_max_ms',
        'shmem_std_ms', 'fallback_std_ms',
        'shmem_outliers', 'fallback_outliers',
        'shmem_cpu_mean_pct', 'fallback_cpu_mean_pct',
        'shmem_cpu_max_pct', 'fallback_cpu_max_pct',
        'shmem_rss_peak_mb', 'fallback_rss_peak_mb',
        'shmem_clock_mean_mhz', 'fallback_clock_mean_mhz',
        'shmem_power_mean_w', 'fallback_power_mean_w',
    ]

    merged_row = [
        resolution, size_bytes, target_hz, str(actual_pub_hz),
        str(shmem_recv_hz), str(fallback_recv_hz),
        str(shmem_frames), str(fallback_frames),
        str(shmem_loss_pct), str(fallback_loss_pct),
        str(shmem_p50), str(fallback_p50), p50_speedup,
        str(shmem_p95), str(fallback_p95),
        str(shmem_p99), str(fallback_p99), p99_speedup,
        str(shmem_mean), str(fallback_mean), mean_speedup,
        str(shmem_min), str(fallback_min),
        str(shmem_max), str(fallback_max),
        str(shmem_std), str(fallback_std),
        str(shmem_outliers), str(fallback_outliers),
        str(shmem_cpu_mean), str(fallback_cpu_mean),
        str(shmem_cpu_max), str(fallback_cpu_max),
        str(shmem_rss_peak), str(fallback_rss_peak),
        str(shmem_clock_mean), str(fallback_clock_mean),
        str(shmem_power_mean), str(fallback_power_mean),
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
    print('\n' + '=' * 105)
    print(f' ZERO-COPY SHMEM vs CPU FALLBACK BENCHMARK SUMMARY  [{resolution} @ {target_hz} Hz]')
    print('=' * 105)
    print(f" {'Metric':<26} | {'SHMEM (Zero-Copy)':<22} | {'Fallback (CPU / Wire)':<22} | {'Speedup / Ratio'}")
    print('-' * 105)
    print(f" {'Actual Publishing Rate':<26} | {actual_pub_hz + ' Hz':<22} | {actual_pub_hz + ' Hz':<22} | Target: {target_hz} Hz")
    print(f" {'Actual Receiving Rate':<26} | {shmem_recv_hz + ' Hz':<22} | {fallback_recv_hz + ' Hz':<22} |")
    print(f" {'Measured Frames':<26} | {shmem_frames:<22} | {fallback_frames:<22} |")
    print(f" {'Lost Frames (Loss %)':<26} | {f'{shmem_lost} ({shmem_loss_pct}%)':<22} | {f'{fallback_lost} ({fallback_loss_pct}%)':<22} |")
    print('-' * 105)
    print(f" {'P50 Latency (Median)':<26} | {shmem_p50 + ' ms':<22} | {fallback_p50 + ' ms':<22} | {p50_speedup}")
    print(f" {'P95 Latency':<26} | {shmem_p95 + ' ms':<22} | {fallback_p95 + ' ms':<22} |")
    print(f" {'P99 Latency':<26} | {shmem_p99 + ' ms':<22} | {fallback_p99 + ' ms':<22} | {p99_speedup}")
    print(f" {'Mean Latency':<26} | {shmem_mean + ' ms':<22} | {fallback_mean + ' ms':<22} | {mean_speedup}")
    print(f" {'Min Latency':<26} | {shmem_min + ' ms':<22} | {fallback_min + ' ms':<22} |")
    print(f" {'Max Latency':<26} | {shmem_max + ' ms':<22} | {fallback_max + ' ms':<22} |")
    print(f" {'Std Dev (Jitter)':<26} | {shmem_std + ' ms':<22} | {fallback_std + ' ms':<22} |")
    print(f" {'Outliers (> 3-sigma)':<26} | {shmem_outliers:<22} | {fallback_outliers:<22} |")
    print('-' * 105)
    print(f" {'System CPU Util (Mean/Max)':<26} | {f'{shmem_cpu_mean}% / {shmem_cpu_max}%':<22} | {f'{fallback_cpu_mean}% / {fallback_cpu_max}%':<22} |")
    print(f" {'Process RSS (Peak)':<26} | {f'{shmem_rss_peak} MB':<22} | {f'{fallback_rss_peak} MB':<22} |")
    print(f" {'CPU Clock (Mean/Max)':<26} | {f'{shmem_clock_mean} / {shmem_clock_max} MHz':<22} | {f'{fallback_clock_mean} / {fallback_clock_max} MHz':<22} |")
    print(f" {'Power Draw (Mean/Max)':<26} | {f'{shmem_power_mean} / {shmem_power_max} W':<22} | {f'{fallback_power_mean} / {fallback_power_max} W':<22} |")
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

    # Temporary CSV paths for the two individual subscriber nodes
    shmem_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".shmem.tmp'"])
    fallback_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".fallback.tmp'"])

    # 1. Subscriber 1: Zero-Copy Shared Memory Subscriber
    subscriber_shared = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='subscriber_shared',
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
            'auto_exit': auto_exit_expr,
        }],
    )

    # 2. Subscriber 2: Fallback Wire (CPU copy) Subscriber
    subscriber_fallback = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='subscriber_fallback',
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
            'auto_exit': auto_exit_expr,
        }],
    )

    # 3. Publisher: Shared Memory Publisher
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
            'auto_exit': auto_exit_expr,
        }],
    )

    delayed_publisher = TimerAction(
        period=1.0,
        actions=[publisher]
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=subscriber_fallback,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Fallback subscriber finished benchmark targets')),
            ]
        )
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
        OpaqueFunction(function=init_csv_file),
        subscriber_shared,
        subscriber_fallback,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
