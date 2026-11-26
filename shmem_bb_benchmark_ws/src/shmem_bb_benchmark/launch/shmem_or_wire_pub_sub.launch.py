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
                pub_cpu_peak = r.get('pub_cpu_peak_pct', 'N/A')
                sub_cpu_mean = r.get('sub_cpu_mean_pct', 'N/A')
                sub_cpu_peak = r.get('sub_cpu_peak_pct', 'N/A')
                pub_pss_mean = r.get('pub_pss_mean_mb', 'N/A')
                pub_pss_peak = r.get('pub_pss_peak_mb', 'N/A')
                sub_pss_mean = r.get('sub_pss_mean_mb', 'N/A')
                sub_pss_peak = r.get('sub_pss_peak_mb', 'N/A')
                print(" System Resources:")
                if pub_cpu_mean != 'N/A':
                    print(f" {'  Publisher CPU':<26} | Mean: {pub_cpu_mean}% | Peak: {pub_cpu_peak}%")
                else:
                    print(f" {'  Publisher CPU':<26} | N/A")
                if sub_cpu_mean != 'N/A':
                    print(f" {'  Subscriber CPU':<26} | Mean: {sub_cpu_mean}% | Peak: {sub_cpu_peak}%")
                else:
                    print(f" {'  Subscriber CPU':<26} | N/A")
                if pub_pss_mean != 'N/A':
                    print(f" {'  Publisher PSS':<26} | Mean: {pub_pss_mean} MB | Peak: {pub_pss_peak} MB")
                else:
                    print(f" {'  Publisher PSS':<26} | N/A")
                if sub_pss_mean != 'N/A':
                    print(f" {'  Subscriber PSS':<26} | Mean: {sub_pss_mean} MB | Peak: {sub_pss_peak} MB")
                else:
                    print(f" {'  Subscriber PSS':<26} | N/A")
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
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')

    # Output and runtime configuration
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/shmem_mode_results.csv',
        description='Path to save output summary CSV results')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool retention and QoS queue')
    latency_csv_path_arg = DeclareLaunchArgument(
        'latency_csv_path', default_value='',
        description='Optional path to save raw per-frame latency CSV')

    # Resolve effective max messages (0 or negative falls back to default 1000)
    effective_max_msgs = PythonExpression([
        "'1000' if int('", LaunchConfiguration('max_msgs'), "') <= 0 else '",
        LaunchConfiguration('max_msgs'), "'"
    ])

    # True when mode is 'shared' or 'shm', False for 'wire' or 'cpu'
    use_shm = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['shared', 'shmem', 'shm', 'true', '1']"
    ])

    subscriber_node = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_subscriber_node',
        name='image_subscriber',
        output='screen',
        parameters=[{
            'topic': 'image_test',
            'result_prefix': 'shmem_bench',
            'use_shm': use_shm,
            'require_shmem': use_shm,
            'verify_content': True,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'latency_csv_path': LaunchConfiguration('latency_csv_path'),
            'auto_exit': True,
        }],
    )

    publisher_node = Node(
        package='shmem_bb_benchmark',
        executable='shmem_image_publisher_node',
        name='image_publisher',
        output='screen',
        parameters=[{
            'topic': 'image_test',
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': use_shm,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'auto_exit': True,
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
        warmup_msgs_arg,
        csv_path_arg,
        history_depth_arg,
        latency_csv_path_arg,
        OpaqueFunction(function=init_csv_file),
        subscriber_node,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
