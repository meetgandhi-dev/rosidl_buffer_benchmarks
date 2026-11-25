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
                # Check if separate callback and buffer-ready latencies are available
                has_cb_br = 'cb_mean_latency_ms' in r and 'br_mean_latency_ms' in r
                if has_cb_br:
                    print(" [1] Subscriber Callback Entry Latency (Pub-to-Callback Entry):")
                    print(f" {'  P50 / P95 / P99':<26} | {r.get('cb_p50_latency_ms','')} / {r.get('cb_p95_latency_ms','')} / {r.get('cb_p99_latency_ms','')} ms")
                    print(f" {'  Mean [Min, Max]':<26} | {r.get('cb_mean_latency_ms','')} [{r.get('cb_min_latency_ms','')}, {r.get('cb_max_latency_ms','')}] ms")
                    print(f" {'  Std Dev (Jitter)':<26} | {r.get('cb_std_dev_ms','')} ms")
                    print(f" {'  Outliers (> 3-sigma)':<26} | {r.get('cb_outliers_3sigma_count','')} frames")
                    print('-' * 105)
                    print(" [2] GPU-Ready Latency (Pub-to-Data Fully Available on Subscriber GPU):")
                    print(f" {'  P50 Latency (Median)':<26} | {r.get('br_p50_latency_ms','')} ms")
                    print(f" {'  P95 Latency':<26} | {r.get('br_p95_latency_ms','')} ms")
                    print(f" {'  P99 Latency':<26} | {r.get('br_p99_latency_ms','')} ms")
                    print(f" {'  Mean Latency':<26} | {r.get('br_mean_latency_ms','')} ms")
                    print(f" {'  Min Latency':<26} | {r.get('br_min_latency_ms','')} ms")
                    print(f" {'  Max Latency':<26} | {r.get('br_max_latency_ms','')} ms")
                    print(f" {'  Std Dev (Jitter)':<26} | {r.get('br_std_dev_ms','')} ms")
                    print(f" {'  Outliers (> 3-sigma)':<26} | {r.get('br_outliers_3sigma_count','')} frames")
                    device_op = r.get('device_op_mean_ms', '')
                    if device_op:
                        print(f" {'  Device Op (H2D / Imp-Sync)':<26} | {device_op} ms")
                    else:
                        try:
                            sync_diff = float(r.get('br_mean_latency_ms', 0)) - float(r.get('cb_mean_latency_ms', 0))
                            print(f" {'  CUDA Sync Overhead':<26} | {max(0.0, sync_diff):.3f} ms")
                        except (ValueError, TypeError):
                            pass
                else:
                    print(f" {'P50 Latency (Median)':<26} | {r.get('p50_latency_ms','')} ms")
                    print(f" {'P95 Latency':<26} | {r.get('p95_latency_ms','')} ms")
                    print(f" {'P99 Latency':<26} | {r.get('p99_latency_ms','')} ms")
                    print(f" {'Mean Latency':<26} | {r.get('mean_latency_ms','')} ms")
                    print(f" {'Min Latency':<26} | {r.get('min_latency_ms','')} ms")
                    print(f" {'Max Latency':<26} | {r.get('max_latency_ms','')} ms")
                    print(f" {'Std Dev (Jitter)':<26} | {r.get('std_dev_ms','')} ms")
                    print(f" {'Outliers (> 3-sigma)':<26} | {r.get('outliers_3sigma_count','')} frames")
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
        'mode', default_value='cuda',
        description="Transport mode: 'cuda' (CUDA IPC zero-copy) or 'wire' (standard DDS CPU copy)")
    max_msgs_arg = DeclareLaunchArgument(
        'max_msgs', default_value='1000',
        description='Maximum messages to publish/receive (default: 1000; set 0 for continuous)')
    max_messages_alias_arg = DeclareLaunchArgument(
        'max_messages', default_value='',
        description='Alias for max_msgs')
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')

    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/cuda_mode_results.csv',
        description='Path to save output summary CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='test_cuda_image',
        description='Topic name for image streaming')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    auto_exit_arg = DeclareLaunchArgument(
        'auto_exit', default_value='',
        description='Whether nodes should automatically exit (default: true when max_msgs > 0)')
    verify_content_arg = DeclareLaunchArgument(
        'verify_content', default_value='false',
        description='Whether to verify payload data integrity (false for performance runs, true for correctness)')
    cuda_device_id_arg = DeclareLaunchArgument(
        'cuda_device_id', default_value='0',
        description='CUDA GPU device ordinal')

    effective_max_msgs = PythonExpression([
        "'", LaunchConfiguration('max_messages'), "' if '",
        LaunchConfiguration('max_messages'), "' != '' else '",
        LaunchConfiguration('max_msgs'), "'"
    ])

    use_cuda = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['cuda', 'gpu', 'true', '1']"
    ])

    auto_exit_expr = PythonExpression([
        "'", LaunchConfiguration('auto_exit'), "' if '",
        LaunchConfiguration('auto_exit'), "' != '' else (",
        "'true' if int('", effective_max_msgs, "') > 0 else 'false')"
    ])

    verify_content_expr = PythonExpression([
        "'", LaunchConfiguration('verify_content'), "'.strip().lower() in ['true', '1', 'yes']"
    ])

    subscriber_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='cuda_image_subscriber',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'cuda_bench',
            'use_cuda': True,
            'require_cuda': use_cuda,
            'verify_content': verify_content_expr,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
    )

    publisher_node = Node(
        package='cuda_bb_benchmark',
        executable='cuda_image_publisher_node',
        name='cuda_image_publisher',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_cuda': use_cuda,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
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
        cuda_device_id_arg,
        OpaqueFunction(function=init_csv_file),
        subscriber_node,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
