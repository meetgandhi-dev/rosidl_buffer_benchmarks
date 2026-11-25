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
    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
            if os.path.exists(csv_file):
                os.remove(csv_file)
        except OSError:
            pass


def print_csv_results(*args, **kwargs):
    context = None
    for a in args:
        if hasattr(a, 'launch_configurations'):
            context = a
            break
    if context is None:
        return

    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file and os.path.exists(csv_file):
        try:
            with open(csv_file, 'r') as f:
                reader = list(csv.DictReader(f))
            if reader:
                r = reader[-1]
                print('\n' + '=' * 105)
                print(f" BENCHMARK RESULTS: mode={r.get('backend','')} [{r.get('resolution','')} @ {r.get('target_hz','')} Hz]")
                print('=' * 105)
                print(f" {'Metric':<26} | {'Value'}")
                print('-' * 105)
                actual_pub_fps = r.get('actual_pub_fps', r.get('actual_pub_hz', ''))
                actual_recv_fps = r.get('actual_recv_fps', r.get('actual_recv_hz', ''))
                total_pub = r.get('total_published_frames', r.get('total_sent', ''))
                total_recv = r.get('total_received_frames', r.get('received_frames', ''))
                dropped = r.get('dropped_frames', r.get('lost_frames', ''))
                loss_pct = r.get('frame_loss_pct', r.get('loss_pct', ''))
                print(f" {'Actual Publishing Rate':<26} | {actual_pub_fps} FPS (Target: {r.get('target_hz','')} Hz)")
                print(f" {'Actual Receiving Rate':<26} | {actual_recv_fps} FPS")
                print(f" {'Measured Frames':<26} | {total_recv} (Total Published: {total_pub})")
                print(f" {'Lost Frames (Loss %)':<26} | {dropped} ({loss_pct}%)")
                print('-' * 105)
                print(" [1] Callback-Entry Latency (Pub-to-Callback Entry):")
                print(f" {'  P50 Latency (Median)':<26} | {r.get('p50_latency_ms','')} ms")
                print(f" {'  P95 Latency':<26} | {r.get('p95_latency_ms','')} ms")
                print(f" {'  P99 Latency':<26} | {r.get('p99_latency_ms','')} ms")
                print(f" {'  Mean Latency':<26} | {r.get('mean_latency_ms','')} ms")
                print(f" {'  Min Latency':<26} | {r.get('min_latency_ms','')} ms")
                print(f" {'  Max Latency':<26} | {r.get('max_latency_ms','')} ms")
                print(f" {'  Std Dev (Jitter)':<26} | {r.get('std_dev_ms','')} ms")
                print(f" {'  Outliers (> 3-sigma)':<26} | {r.get('outliers_3sigma_count','')} frames")
                if 'data_ready_p50_ms' in r:
                    print('-' * 105)
                    print(" [2] Backend Data-Ready Latency (Pub-to-Payload Pointer Accessible):")
                    print(f" {'  P50 Data-Ready':<26} | {r.get('data_ready_p50_ms','')} ms")
                    print(f" {'  P95 Data-Ready':<26} | {r.get('data_ready_p95_ms','')} ms")
                    print(f" {'  P99 Data-Ready':<26} | {r.get('data_ready_p99_ms','')} ms")
                    print(f" {'  Mean Data-Ready':<26} | {r.get('data_ready_mean_ms','')} ms")
                    print(f" {'  Std Dev Data-Ready':<26} | {r.get('data_ready_std_dev_ms','')} ms")
                print('-' * 105)
                pub_cpu_mean = r.get('pub_cpu_mean_pct', 'N/A')
                pub_cpu_max = r.get('pub_cpu_max_pct', 'N/A')
                sub_cpu_mean = r.get('sub_cpu_mean_pct', r.get('cpu_util_mean_pct', 'N/A'))
                sub_cpu_max = r.get('sub_cpu_max_pct', r.get('cpu_util_max_pct', 'N/A'))
                pub_rss_peak = r.get('pub_rss_peak_mb', 'N/A')
                sub_rss_peak = r.get('sub_rss_peak_mb', r.get('process_rss_peak_mb', 'N/A'))
                sys_ram = r.get('system_ram_mb', 'N/A')
                gpu_util = r.get('gpu_util_pct', 'N/A')
                gpu_freq = r.get('gpu_freq_mhz', 'N/A')
                emc_util = r.get('emc_util_pct', 'N/A')
                emc_freq = r.get('emc_freq_mhz', 'N/A')
                cpu_temp = r.get('cpu_temp_c', 'N/A')
                gpu_temp = r.get('gpu_temp_c', 'N/A')
                print(f" {'Publisher CPU Util':<26} | Mean: {pub_cpu_mean}% | Max: {pub_cpu_max}%")
                print(f" {'Subscriber CPU Util':<26} | Mean: {sub_cpu_mean}% | Max: {sub_cpu_max}%")
                print(f" {'Process RSS Peak':<26} | Pub: {pub_rss_peak} MB | Sub: {sub_rss_peak} MB")
                print(f" {'System RAM Used':<26} | {sys_ram} MB")
                if gpu_util != 'N/A':
                    gpu_str = f'{gpu_util}%' + (f' @ {gpu_freq} MHz' if gpu_freq != 'N/A' else '')
                    print(f" {'GPU Util / Freq':<26} | {gpu_str}")
                if emc_util != 'N/A' or emc_freq != 'N/A':
                    emc_str = f'{emc_util}%' + (f' @ {emc_freq} MHz' if emc_freq != 'N/A' else '')
                    print(f" {'EMC Memory Controller':<26} | {emc_str}")
                if cpu_temp != 'N/A' or gpu_temp != 'N/A':
                    cpu_t_str = f'{cpu_temp} C' if cpu_temp != 'N/A' else 'N/A'
                    gpu_t_str = f'{gpu_temp} C' if gpu_temp != 'N/A' else 'N/A'
                    temp_str = f'CPU: {cpu_t_str} | GPU: {gpu_t_str}'
                    print(f" {'Temperatures':<26} | {temp_str}")
                print('=' * 105)
                print(f' Results saved to: {csv_file}\n')
        except OSError:
            pass


def generate_launch_description():
    # Primary arguments: width, height, rate, mode, and max_msgs
    width_arg = DeclareLaunchArgument(
        'width', default_value='1920',
        description='Image width in pixels')
    height_arg = DeclareLaunchArgument(
        'height', default_value='1080',
        description='Image height in pixels')
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='30.0',
        description='Publishing rate in Hz')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='shared',
        description="Transport mode: 'shared' (SHM zero-copy) or 'wire' (standard DDS CPU copy)")
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
        'csv_path', default_value='/tmp/shmem_mode_results.csv',
        description='Path to save output summary CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='image_test',
        description='Topic name for image streaming')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    auto_exit_arg = DeclareLaunchArgument(
        'auto_exit', default_value='',
        description='Whether nodes should automatically exit (default: true when max_msgs > 0)')
    verify_content_arg = DeclareLaunchArgument(
        'verify_content', default_value='false',
        description='Whether subscriber verifies pixel payload byte patterns (false for performance runs, true for correctness)')

    # Resolve effective max messages
    effective_max_msgs = PythonExpression([
        "'", LaunchConfiguration('max_messages'), "' if '",
        LaunchConfiguration('max_messages'), "' != '' else '",
        LaunchConfiguration('max_msgs'), "'"
    ])

    # True when mode is 'shared' or 'shm', False for 'wire' or 'cpu'
    use_shm = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['shared', 'shmem', 'shm', 'true', '1']"
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

    subscriber_node = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='image_subscriber',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'shmem_bench',
            'use_shm': use_shm,
            'require_shmem': use_shm,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'auto_exit': auto_exit_expr,
        }],
    )

    publisher_node = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_publisher_node',
        name='image_publisher',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': use_shm,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'auto_exit': auto_exit_expr,
        }],
    )

    delayed_publisher = TimerAction(
        period=1.0,
        actions=[publisher_node]
    )

    # When subscriber exits, initiate clean shutdown and display CSV
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=subscriber_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Subscriber finished benchmark targets')),
            ]
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=print_csv_results)
            ]
        )
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        rate_arg,
        mode_arg,
        max_msgs_arg,
        max_messages_alias_arg,
        warmup_msgs_arg,
        csv_path_arg,
        topic_arg,
        history_depth_arg,
        auto_exit_arg,
        verify_content_arg,
        OpaqueFunction(function=init_csv_file),
        subscriber_node,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
