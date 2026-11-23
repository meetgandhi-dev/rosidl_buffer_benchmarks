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


def merge_and_print_csv_results(event, context):
    """Merge shmem and fallback results into a single comparison CSV and print summary."""
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

    shmem_frames = shmem_data.get('received_frames', '0') if shmem_data else '0'
    shmem_avg = shmem_data.get('avg_latency_ms', '0.0') if shmem_data else '0.0'
    shmem_min = shmem_data.get('min_latency_ms', '0.0') if shmem_data else '0.0'
    shmem_max = shmem_data.get('max_latency_ms', '0.0') if shmem_data else '0.0'
    shmem_std = shmem_data.get('std_dev_ms', '0.0') if shmem_data else '0.0'

    fallback_frames = fallback_data.get('received_frames', '0') if fallback_data else '0'
    fallback_avg = fallback_data.get('avg_latency_ms', '0.0') if fallback_data else '0.0'
    fallback_min = fallback_data.get('min_latency_ms', '0.0') if fallback_data else '0.0'
    fallback_max = fallback_data.get('max_latency_ms', '0.0') if fallback_data else '0.0'
    fallback_std = fallback_data.get('std_dev_ms', '0.0') if fallback_data else '0.0'

    diff_ms_str = 'N/A'
    speedup_str = 'N/A'
    diff_ms_val = 0.0
    try:
        shmem_avg_f = float(shmem_avg)
        fallback_avg_f = float(fallback_avg)
        diff_ms_val = fallback_avg_f - shmem_avg_f
        diff_ms_str = f'{diff_ms_val:.3f}'
        if shmem_avg_f > 0.0:
            speedup_ratio = fallback_avg_f / shmem_avg_f
            speedup_str = f'{speedup_ratio:.2f}x'
    except (ValueError, TypeError):
        pass

    merged_header = [
        'resolution', 'size_bytes', 'target_hz', 'frame_cap',
        'shmem_frames', 'fallback_frames',
        'shmem_avg_latency_ms', 'fallback_avg_latency_ms',
        'latency_diff_ms', 'speedup',
        'shmem_min_latency_ms', 'fallback_min_latency_ms',
        'shmem_max_latency_ms', 'fallback_max_latency_ms',
        'shmem_std_dev_ms', 'fallback_std_dev_ms',
    ]

    merged_row = [
        resolution,
        size_bytes,
        target_hz,
        frame_cap,
        shmem_frames,
        fallback_frames,
        shmem_avg,
        fallback_avg,
        diff_ms_str,
        speedup_str,
        shmem_min,
        fallback_min,
        shmem_max,
        fallback_max,
        shmem_std,
        fallback_std,
    ]

    header_str = ','.join(merged_header) + '\n'
    row_str = ','.join(merged_row) + '\n'

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
                f.write(header_str)
            f.write(row_str)
    except OSError as err:
        print(f'Error writing merged CSV to {csv_file}: {err}')

    # Output formatted comparison table and merged CSV to console
    print('\n' + '=' * 88)
    print(' FINAL BENCHMARK COMPARISON RESULTS (2 SUBSCRIBERS: SHMEM vs FALLBACK):')
    print('=' * 88)
    print(f' Resolution       : {resolution} ({size_bytes} Bytes)')
    print(f' Target Rate      : {target_hz} Hz (Frame Cap: {frame_cap})')
    print('-' * 88)
    print(f" {'Metric':<22} | {'SHMEM (Zero-Copy)':<26} | {'Fallback (CPU / Wire)'}")
    print('-' * 88)
    print(f" {'Received Frames':<22} | {shmem_frames:<26} | {fallback_frames}")
    print(f" {'Avg Latency':<22} | {shmem_avg + ' ms':<26} | {fallback_avg + ' ms'}")
    print(f" {'Min Latency':<22} | {shmem_min + ' ms':<26} | {fallback_min + ' ms'}")
    print(f" {'Max Latency':<22} | {shmem_max + ' ms':<26} | {fallback_max + ' ms'}")
    print(f" {'Std Dev':<22} | {shmem_std + ' ms':<26} | {fallback_std + ' ms'}")
    print('-' * 88)
    if speedup_str != 'N/A' and diff_ms_val >= 0:
        print(f' Comparison       : SHMEM is {speedup_str} faster (-{diff_ms_str} ms latency)')
    elif speedup_str != 'N/A':
        print(f' Comparison       : SHMEM is {speedup_str} (latency difference: {diff_ms_str} ms)')
    print('=' * 88)
    print(' MERGED CSV OUTPUT:')
    print('=' * 88)
    try:
        with open(csv_file, 'r') as f:
            print(f.read().strip())
    except OSError:
        print(row_str.strip())
    print('=' * 88)
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

    # Output and runtime configuration
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/shmem_fallback_results.csv',
        description='Path to save output merged comparison CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='image_test',
        description='Topic name for image streaming')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    auto_exit_arg = DeclareLaunchArgument(
        'auto_exit', default_value='',
        description='Whether nodes should automatically exit (default: true when max_msgs > 0)')

    # Resolve effective max messages
    effective_max_msgs = PythonExpression([
        "int('", LaunchConfiguration('max_messages'), "') if '",
        LaunchConfiguration('max_messages'), "'.strip() != '' else int('",
        LaunchConfiguration('max_msgs'), "')"
    ])

    # Automatically enable auto_exit if max_msgs > 0 unless explicitly overridden
    auto_exit_expr = PythonExpression([
        "'true' if ('", LaunchConfiguration('auto_exit'),
        "'.strip().lower() in ['true', '1'] if '",
        LaunchConfiguration('auto_exit'), "'.strip() != '' else (int('",
        LaunchConfiguration('max_messages'), "') > 0 if '",
        LaunchConfiguration('max_messages'), "'.strip() != '' else int('",
        LaunchConfiguration('max_msgs'), "') > 0)) else 'false'"
    ])

    # 1. Shared-memory publisher node
    pub_node = Node(
        package='shmem_buffer_backend_test',
        executable='shmem_image_publisher_node',
        name='publisher_shared',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': True,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'auto_exit': auto_exit_expr,
        }],
        output='screen',
    )

    # 2. Shared-memory subscriber node (zero-copy POSIX shared memory)
    sub_shared_node = Node(
        package='shmem_buffer_backend_test',
        executable='shmem_image_subscriber_node',
        name='subscriber_shared',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'sub_shared',
            'use_shm': True,
            'require_shmem': True,
            'verify_content': True,
            'max_messages': effective_max_msgs,
            'history_depth': LaunchConfiguration('history_depth'),
            'publish_diagnostics': False,
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': PythonExpression([
                "'", LaunchConfiguration('csv_path'), ".shmem.tmp'"
            ]),
            'auto_exit': auto_exit_expr,
        }],
        output='screen',
    )

    # 3. Fallback / wire subscriber node (standard DDS CDR serialized copy)
    sub_fallback_node = Node(
        package='shmem_buffer_backend_test',
        executable='shmem_image_subscriber_node',
        name='subscriber_fallback',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'sub_fallback',
            'use_shm': False,
            'require_shmem': False,
            'verify_content': True,
            'max_messages': effective_max_msgs,
            'history_depth': LaunchConfiguration('history_depth'),
            'publish_diagnostics': False,
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': PythonExpression([
                "'", LaunchConfiguration('csv_path'), ".fallback.tmp'"
            ]),
            'auto_exit': auto_exit_expr,
        }],
        output='screen',
    )

    # Delay publisher startup slightly so both subscribers finish DDS discovery
    delayed_pub_node = TimerAction(
        period=0.5,
        actions=[pub_node],
    )

    # When publisher exits after max_msgs, allow 2.5s for subscribers to drain, then shut down
    shutdown_on_pub_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=pub_node,
            on_exit=[
                TimerAction(
                    period=2.5,
                    actions=[EmitEvent(event=Shutdown(reason='Publisher finished publication'))],
                ),
            ],
        )
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        rate_arg,
        max_msgs_arg,
        max_messages_alias_arg,
        csv_path_arg,
        topic_arg,
        history_depth_arg,
        auto_exit_arg,
        OpaqueFunction(function=init_csv_file),
        sub_shared_node,
        sub_fallback_node,
        delayed_pub_node,
        shutdown_on_pub_exit,
        RegisterEventHandler(OnShutdown(on_shutdown=merge_and_print_csv_results)),
    ])
