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


def merge_and_print_csv_results(event, context):
    """Merge cuda and fallback results into a single comparison CSV and print summary."""
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

    cuda_frames = cuda_data.get('received_frames', '0') if cuda_data else '0'
    cuda_mean = float(cuda_data['avg_latency_ms']) if cuda_data and 'avg_latency_ms' in cuda_data else None
    cuda_min = float(cuda_data['min_latency_ms']) if cuda_data and 'min_latency_ms' in cuda_data else None
    cuda_max = float(cuda_data['max_latency_ms']) if cuda_data and 'max_latency_ms' in cuda_data else None
    cuda_std = float(cuda_data['std_dev_ms']) if cuda_data and 'std_dev_ms' in cuda_data else None

    fb_frames = fallback_data.get('received_frames', '0') if fallback_data else '0'
    fb_mean = float(fallback_data['avg_latency_ms']) if fallback_data and 'avg_latency_ms' in fallback_data else None
    fb_min = float(fallback_data['min_latency_ms']) if fallback_data and 'min_latency_ms' in fallback_data else None
    fb_max = float(fallback_data['max_latency_ms']) if fallback_data and 'max_latency_ms' in fallback_data else None
    fb_std = float(fallback_data['std_dev_ms']) if fallback_data and 'std_dev_ms' in fallback_data else None

    speedup_str = 'N/A'
    if cuda_mean is not None and fb_mean is not None and cuda_mean > 0:
        ratio = fb_mean / cuda_mean
        speedup_str = f'{ratio:.2f}x'

    def fmt(v):
        return f'{v:.3f}' if v is not None else 'N/A'

    # Append row to final comparison CSV
    try:
        exists = os.path.exists(csv_file) and os.path.getsize(csv_file) > 0
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not exists:
                writer.writerow([
                    'resolution', 'size_bytes', 'target_hz', 'frame_cap',
                    'cuda_received', 'cuda_mean_ms', 'cuda_min_ms', 'cuda_max_ms', 'cuda_std_ms',
                    'fallback_received', 'fallback_mean_ms', 'fallback_min_ms', 'fallback_max_ms', 'fallback_std_ms',
                    'speedup'
                ])
            writer.writerow([
                resolution, size_bytes, target_hz, frame_cap,
                cuda_frames, fmt(cuda_mean), fmt(cuda_min), fmt(cuda_max), fmt(cuda_std),
                fb_frames, fmt(fb_mean), fmt(fb_min), fmt(fb_max), fmt(fb_std),
                speedup_str
            ])
    except OSError:
        pass

    # Print summary table to terminal
    print('\n' + '=' * 105)
    print(f' CUDA ZERO-COPY vs CPU FALLBACK BENCHMARK SUMMARY  [{resolution} @ {target_hz} Hz]')
    print('=' * 105)
    print(f' {"Metric":<26} | {"CUDA Zero-Copy":<20} | {"CPU Fallback (Wire)":<22} | {"Speedup":<12}')
    print('-' * 105)
    print(f' {"Frames Measured":<26} | {cuda_frames:<20} | {fb_frames:<22} |')
    print(f' {"Mean Latency (ms)":<26} | {fmt(cuda_mean):<20} | {fmt(fb_mean):<22} | {speedup_str:<12}')
    print(f' {"Min Latency (ms)":<26} | {fmt(cuda_min):<20} | {fmt(fb_min):<22} |')
    print(f' {"Max Latency (ms)":<26} | {fmt(cuda_max):<20} | {fmt(fb_max):<22} |')
    print(f' {"Std Dev (ms)":<26} | {fmt(cuda_std):<20} | {fmt(fb_std):<22} |')
    print('=' * 105)
    print(f' Results appended to: {csv_file}\n')


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

    # auto_exit: True when max_msgs > 0
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

    # When fallback subscriber exits (usually last or equal), shut down
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
