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
            for ext in ['.cuda.tmp', '.fallback.tmp']:
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

    # Wait briefly for subscriber nodes to finalize writing summary files
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if os.path.exists(cuda_tmp) and os.path.exists(fallback_tmp):
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

    cuda_data = read_summary(cuda_tmp)
    fallback_data = read_summary(fallback_tmp)

    # Clean up temporary files
    for tmp in [cuda_tmp, fallback_tmp]:
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
    frame_cap = src.get('frame_cap', context.launch_configurations.get('max_msgs', ''))

    actual_pub_hz = cuda_data.get('actual_pub_hz', src.get('actual_pub_hz', target_hz)) if cuda_data else '0.0'
    cuda_recv_hz = cuda_data.get('actual_recv_hz', '0.0') if cuda_data else '0.0'
    fallback_recv_hz = fallback_data.get('actual_recv_hz', '0.0') if fallback_data else '0.0'

    cuda_frames = cuda_data.get('received_frames', '0') if cuda_data else '0'
    fallback_frames = fallback_data.get('received_frames', '0') if fallback_data else '0'

    cuda_lost = cuda_data.get('lost_frames', '0') if cuda_data else '0'
    fallback_lost = fallback_data.get('lost_frames', '0') if fallback_data else '0'

    cuda_loss_pct = cuda_data.get('loss_pct', '0.0') if cuda_data else '0.0'
    fallback_loss_pct = fallback_data.get('loss_pct', '0.0') if fallback_data else '0.0'

    cuda_p50 = cuda_data.get('p50_latency_ms', '0.0') if cuda_data else '0.0'
    fallback_p50 = fallback_data.get('p50_latency_ms', '0.0') if fallback_data else '0.0'

    cuda_p95 = cuda_data.get('p95_latency_ms', '0.0') if cuda_data else '0.0'
    fallback_p95 = fallback_data.get('p95_latency_ms', '0.0') if fallback_data else '0.0'

    cuda_p99 = cuda_data.get('p99_latency_ms', '0.0') if cuda_data else '0.0'
    fallback_p99 = fallback_data.get('p99_latency_ms', '0.0') if fallback_data else '0.0'

    cuda_mean = cuda_data.get('mean_latency_ms', cuda_data.get('avg_latency_ms', '0.0')) if cuda_data else '0.0'
    fallback_mean = fallback_data.get('mean_latency_ms', fallback_data.get('avg_latency_ms', '0.0')) if fallback_data else '0.0'

    cuda_min = cuda_data.get('min_latency_ms', '0.0') if cuda_data else '0.0'
    fallback_min = fallback_data.get('min_latency_ms', '0.0') if fallback_data else '0.0'

    cuda_max = cuda_data.get('max_latency_ms', '0.0') if cuda_data else '0.0'
    fallback_max = fallback_data.get('max_latency_ms', '0.0') if fallback_data else '0.0'

    cuda_std = cuda_data.get('std_dev_ms', '0.0') if cuda_data else '0.0'
    fallback_std = fallback_data.get('std_dev_ms', '0.0') if fallback_data else '0.0'

    cuda_outliers = cuda_data.get('outliers_3sigma_count', '0') if cuda_data else '0'
    fallback_outliers = fallback_data.get('outliers_3sigma_count', '0') if fallback_data else '0'

    def calc_speedup(fb_str, cuda_str):
        try:
            fb = float(fb_str)
            cu = float(cuda_str)
            if cu > 0.0:
                return f'{fb / cu:.2f}x'
        except (ValueError, TypeError):
            pass
        return 'N/A'

    p50_speedup = calc_speedup(fallback_p50, cuda_p50)
    p99_speedup = calc_speedup(fallback_p99, cuda_p99)
    mean_speedup = calc_speedup(fallback_mean, cuda_mean)

    merged_header = [
        'resolution', 'size_bytes', 'target_hz', 'actual_pub_hz',
        'cuda_recv_hz', 'fallback_recv_hz',
        'cuda_frames', 'fallback_frames',
        'cuda_loss_pct', 'fallback_loss_pct',
        'cuda_p50_ms', 'fallback_p50_ms', 'p50_speedup',
        'cuda_p95_ms', 'fallback_p95_ms',
        'cuda_p99_ms', 'fallback_p99_ms', 'p99_speedup',
        'cuda_mean_ms', 'fallback_mean_ms', 'mean_speedup',
        'cuda_min_ms', 'fallback_min_ms',
        'cuda_max_ms', 'fallback_max_ms',
        'cuda_std_ms', 'fallback_std_ms',
        'cuda_outliers', 'fallback_outliers',
    ]

    merged_row = [
        resolution, size_bytes, target_hz, str(actual_pub_hz),
        str(cuda_recv_hz), str(fallback_recv_hz),
        str(cuda_frames), str(fallback_frames),
        str(cuda_loss_pct), str(fallback_loss_pct),
        str(cuda_p50), str(fallback_p50), p50_speedup,
        str(cuda_p95), str(fallback_p95),
        str(cuda_p99), str(fallback_p99), p99_speedup,
        str(cuda_mean), str(fallback_mean), mean_speedup,
        str(cuda_min), str(fallback_min),
        str(cuda_max), str(fallback_max),
        str(cuda_std), str(fallback_std),
        str(cuda_outliers), str(fallback_outliers),
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
    print(f' CUDA ZERO-COPY vs CPU FALLBACK BENCHMARK SUMMARY  [{resolution} @ {target_hz} Hz]')
    print('=' * 105)
    print(f" {'Metric':<26} | {'CUDA (Zero-Copy)':<22} | {'Fallback (CPU / Wire)':<22} | {'Speedup / Ratio'}")
    print('-' * 105)
    print(f" {'Actual Publishing Rate':<26} | {actual_pub_hz + ' Hz':<22} | {actual_pub_hz + ' Hz':<22} | Target: {target_hz} Hz")
    print(f" {'Actual Receiving Rate':<26} | {cuda_recv_hz + ' Hz':<22} | {fallback_recv_hz + ' Hz':<22} |")
    print(f" {'Measured Frames':<26} | {cuda_frames:<22} | {fallback_frames:<22} |")
    print(f" {'Lost Frames (Loss %)':<26} | {f'{cuda_lost} ({cuda_loss_pct}%)':<22} | {f'{fallback_lost} ({fallback_loss_pct}%)':<22} |")
    print('-' * 105)
    print(f" {'P50 Latency (Median)':<26} | {cuda_p50 + ' ms':<22} | {fallback_p50 + ' ms':<22} | {p50_speedup}")
    print(f" {'P95 Latency':<26} | {cuda_p95 + ' ms':<22} | {fallback_p95 + ' ms':<22} |")
    print(f" {'P99 Latency':<26} | {cuda_p99 + ' ms':<22} | {fallback_p99 + ' ms':<22} | {p99_speedup}")
    print(f" {'Mean Latency':<26} | {cuda_mean + ' ms':<22} | {fallback_mean + ' ms':<22} | {mean_speedup}")
    print(f" {'Min Latency':<26} | {cuda_min + ' ms':<22} | {fallback_min + ' ms':<22} |")
    print(f" {'Max Latency':<26} | {cuda_max + ' ms':<22} | {fallback_max + ' ms':<22} |")
    print(f" {'Std Dev (Jitter)':<26} | {cuda_std + ' ms':<22} | {fallback_std + ' ms':<22} |")
    print(f" {'Outliers (> 3-sigma)':<26} | {cuda_outliers:<22} | {fallback_outliers:<22} |")
    print('=' * 105)
    print(f' Results saved to: {csv_file}\n')


def generate_launch_description():
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
        'csv_path', default_value='/tmp/cuda_fallback_comparison.csv',
        description='Path to save output comparison CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='test_cuda_image',
        description='Topic name for image streaming')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    verify_content_arg = DeclareLaunchArgument(
        'verify_content', default_value='true',
        description='Whether to verify payload data integrity')
    cuda_device_id_arg = DeclareLaunchArgument(
        'cuda_device_id', default_value='0',
        description='CUDA GPU device ordinal')

    auto_exit_expr = PythonExpression([
        "'true' if int('", LaunchConfiguration('max_msgs'), "') > 0 else 'false'"
    ])

    verify_content_expr = PythonExpression([
        "'", LaunchConfiguration('verify_content'), "'.strip().lower() in ['true', '1', 'yes']"
    ])

    cuda_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".cuda.tmp'"])
    fallback_tmp_csv = PythonExpression(["'", LaunchConfiguration('csv_path'), ".fallback.tmp'"])

    cuda_subscriber_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='cuda_subscriber',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'cuda_sub',
            'use_cuda': True,
            'require_cuda': True,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': cuda_tmp_csv,
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    fallback_subscriber_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='fallback_subscriber',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'fallback_sub',
            'use_cuda': False,
            'require_cuda': False,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': fallback_tmp_csv,
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    publisher_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_publisher_node',
        name='cuda_publisher',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_cuda': True,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    delayed_publisher = TimerAction(
        period=1.0,
        actions=[publisher_node]
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fallback_subscriber_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Benchmark completed')),
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
        warmup_msgs_arg,
        csv_path_arg,
        topic_arg,
        history_depth_arg,
        verify_content_arg,
        cuda_device_id_arg,
        OpaqueFunction(function=init_csv_file),
        cuda_subscriber_node,
        fallback_subscriber_node,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
