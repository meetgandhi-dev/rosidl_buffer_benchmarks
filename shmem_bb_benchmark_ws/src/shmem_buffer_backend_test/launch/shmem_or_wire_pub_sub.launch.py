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
        'mode', default_value='shared',
        description="Transport mode: 'shared' (SHM zero-copy) or 'wire' (standard DDS CPU copy)")
    max_msgs_arg = DeclareLaunchArgument(
        'max_msgs', default_value='1000',
        description='Maximum messages to publish/receive (default: 1000; set 0 for continuous)')
    max_messages_alias_arg = DeclareLaunchArgument(
        'max_messages', default_value='',
        description='Alias for max_msgs')

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

    # Boolean expression for mode: True for 'shared'/'shm'/'true', False for 'wire'/'cpu'/'false'
    use_shm = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['shared', 'shm', 'true', '1']"
    ])

    # 1. Publisher node
    pub_node = Node(
        package='shmem_buffer_backend_test',
        executable='shmem_image_publisher_node',
        name='image_publisher',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_shm': use_shm,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': effective_max_msgs,
            'auto_exit': auto_exit_expr,
        }],
        output='screen',
    )

    # 2. Subscriber node
    sub_node = Node(
        package='shmem_buffer_backend_test',
        executable='shmem_image_subscriber_node',
        name='image_subscriber',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'image_sub',
            'use_shm': use_shm,
            'require_shmem': use_shm,
            'verify_content': True,
            'max_messages': effective_max_msgs,
            'history_depth': LaunchConfiguration('history_depth'),
            'publish_diagnostics': False,
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'auto_exit': auto_exit_expr,
        }],
        output='screen',
    )

    # Delay publisher startup slightly so subscriber finishes DDS discovery
    delayed_pub_node = TimerAction(
        period=0.5,
        actions=[pub_node],
    )

    # When publisher exits after max_msgs, allow 2.5s for subscriber to drain, then shut down
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
        mode_arg,
        max_msgs_arg,
        max_messages_alias_arg,
        csv_path_arg,
        topic_arg,
        history_depth_arg,
        auto_exit_arg,
        OpaqueFunction(function=init_csv_file),
        sub_node,
        delayed_pub_node,
        shutdown_on_pub_exit,
        RegisterEventHandler(OnShutdown(on_shutdown=print_csv_results)),
    ])
