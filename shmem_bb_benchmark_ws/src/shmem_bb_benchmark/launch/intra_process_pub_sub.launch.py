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

"""Intra-process benchmark: pub and sub run in the SAME process via ComposableNodeContainer.

Both nodes share the same address space. With ROS 2 intra-process comms enabled,
the message shared_ptr is delivered directly to the subscriber without any
serialization, DDS discovery, or inter-process copy.

Two backend modes are compared:
  - shmem : publisher allocates payload in shared memory via shmem_buffer_backend.
             Subscriber receives via zero-copy shmem read handle (intra + SHM).
  - cpu   : publisher allocates payload in a standard CPU buffer.
             Subscriber receives a plain CPU buffer (intra + CPU).
"""

import csv
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def init_csv_file(context):
    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file:
        try:
            os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
            with open(csv_file, 'w') as f:
                f.write(
                    'backend,size_bytes,resolution,target_hz,actual_pub_hz,actual_recv_hz,'
                    'total_sent,received_frames,lost_frames,loss_pct,mean_latency_ms,'
                    'min_latency_ms,p50_latency_ms,p95_latency_ms,p99_latency_ms,max_latency_ms,'
                    'std_dev_ms,outliers_3sigma_count,'
                    'cpu_util_mean_pct,cpu_util_max_pct,process_rss_mean_mb,process_rss_peak_mb,'
                    'system_ram_mb,cpu_clock_mean_mhz,cpu_clock_max_mhz,power_mean_w,power_max_w,'
                    'gpu_util_pct,gpu_clock_mhz,gpu_power_w\n')
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
                mode = context.launch_configurations.get('mode', 'shmem')
                print('\n' + '=' * 105)
                print(f" INTRA-PROCESS BENCHMARK RESULTS (mode={mode}): [{r.get('resolution','')} @ {r.get('target_hz','')} Hz]")
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
                print(f" {'P50 Latency (Median)':<26} | {r.get('p50_latency_ms','')} ms")
                print(f" {'P95 Latency':<26} | {r.get('p95_latency_ms','')} ms")
                print(f" {'P99 Latency':<26} | {r.get('p99_latency_ms','')} ms")
                print(f" {'Mean Latency':<26} | {r.get('mean_latency_ms','')} ms")
                print(f" {'Min Latency':<26} | {r.get('min_latency_ms','')} ms")
                print(f" {'Max Latency':<26} | {r.get('max_latency_ms','')} ms")
                print(f" {'Std Dev (Jitter)':<26} | {r.get('std_dev_ms','')} ms")
                print(f" {'Outliers (> 3-sigma)':<26} | {r.get('outliers_3sigma_count','')} frames")
                print('-' * 105)
                cpu_mean = r.get('sub_cpu_mean_pct', r.get('cpu_util_mean_pct', 'N/A'))
                cpu_max = r.get('sub_cpu_max_pct', r.get('cpu_util_max_pct', 'N/A'))
                rss_peak = r.get('sub_rss_peak_mb', r.get('process_rss_peak_mb', 'N/A'))
                sys_ram = r.get('system_ram_mb', 'N/A')
                gpu_util = r.get('gpu_util_pct', 'N/A')
                gpu_freq = r.get('gpu_freq_mhz', 'N/A')
                emc_util = r.get('emc_util_pct', 'N/A')
                emc_freq = r.get('emc_freq_mhz', 'N/A')
                cpu_temp = r.get('cpu_temp_c', 'N/A')
                gpu_temp = r.get('gpu_temp_c', 'N/A')
                print(f" {'Container CPU Util':<26} | Mean: {cpu_mean}% | Max: {cpu_max}%")
                print(f" {'Process RSS Peak':<26} | Container: {rss_peak} MB")
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
        'mode', default_value='shmem',
        description=(
            "Backend mode for intra-process benchmark: "
            "'shmem' (shared memory buffer) or 'cpu' (standard CPU buffer)"))
    max_msgs_arg = DeclareLaunchArgument(
        'max_msgs', default_value='1000',
        description='Number of messages to publish/receive (0 = continuous)')
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/intra_process_results.csv',
        description='Path to save output summary CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='intra_image_test',
        description='Topic name (used only internally within the container)')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool and QoS queue')

    use_shm = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['shmem', 'shm', 'true', '1']"
    ])

    auto_exit_expr = PythonExpression([
        "'true' if int('", LaunchConfiguration('max_msgs'), "') > 0 else 'false'"
    ])

    publisher_node = ComposableNode(
        package='shmem_bb_benchmark',
        plugin='shmem_bb_benchmark::ShmemImagePublisherNode',
        name='intra_image_publisher',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': use_shm,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'auto_exit': auto_exit_expr,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    subscriber_node = ComposableNode(
        package='shmem_bb_benchmark',
        plugin='shmem_bb_benchmark::ShmemImageSubscriberNode',
        name='intra_image_subscriber',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'shmem_intra',
            'use_shm': use_shm,
            'require_shmem': use_shm,
            'verify_content': False,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'auto_exit': auto_exit_expr,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='intra_process_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            publisher_node,
            subscriber_node,
        ],
        output='screen',
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=container,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Intra-process container exited cleanly')),
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
        topic_arg,
        history_depth_arg,
        OpaqueFunction(function=init_csv_file),
        container,
        exit_event_handler,
        shutdown_event_handler,
    ])
