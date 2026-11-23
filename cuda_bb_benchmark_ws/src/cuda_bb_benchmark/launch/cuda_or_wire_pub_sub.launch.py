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
            with open(csv_file, 'w') as f:
                f.write(
                    'backend,size_bytes,resolution,target_hz,frame_cap,received_frames,'
                    'avg_latency_ms,min_latency_ms,max_latency_ms,std_dev_ms\n')
        except OSError:
            pass


def print_csv_results(event, context):
    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file and os.path.exists(csv_file):
        try:
            with open(csv_file, 'r') as f:
                content = f.read().strip()
            if content:
                print('\n' + '=' * 88)
                print(' FINAL BENCHMARK RESULTS (CSV FORMAT):')
                print('=' * 88)
                print(content)
                print('=' * 88)
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

    # Output and runtime configuration
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
        'verify_content', default_value='true',
        description='Whether to verify payload data integrity')
    cuda_device_id_arg = DeclareLaunchArgument(
        'cuda_device_id', default_value='0',
        description='CUDA GPU device ordinal')

    # Resolve effective max messages
    effective_max_msgs = PythonExpression([
        "'", LaunchConfiguration('max_messages'), "' if '",
        LaunchConfiguration('max_messages'), "' != '' else '",
        LaunchConfiguration('max_msgs'), "'"
    ])

    # True when mode is 'cuda', False when mode is 'wire' or 'cpu'
    use_cuda = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['cuda', 'gpu', 'true', '1']"
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
        package='cuda_bb_benchmark',
        executable='cuda_image_subscriber_node',
        name='cuda_image_subscriber',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'cuda_bench',
            'use_cuda': use_cuda,
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
        cuda_device_id_arg,
        OpaqueFunction(function=init_csv_file),
        subscriber_node,
        delayed_publisher,
        exit_event_handler,
        shutdown_event_handler,
    ])
