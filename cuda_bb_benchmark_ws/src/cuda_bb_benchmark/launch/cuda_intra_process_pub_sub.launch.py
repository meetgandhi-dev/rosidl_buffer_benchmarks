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
the message shared_ptr is delivered directly to the subscriber without serialization,
DDS discovery, or inter-process IPC overhead.

Two backend modes can be compared:
  - cuda : publisher allocates payload via cuda_buffer_backend.
           Subscriber receives via CUDA buffer handle (intra + CUDA).
  - cpu  : publisher allocates payload in standard CPU buffer.
           Subscriber receives a plain CPU buffer (intra + CPU).

Usage:
  # Intra-process with CUDA backend:
  ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
    width:=1920 height:=1080 rate:=30.0 mode:=cuda max_msgs:=1000

  # Intra-process with CPU backend:
  ros2 launch cuda_bb_benchmark cuda_intra_process_pub_sub.launch.py \
    width:=1920 height:=1080 rate:=30.0 mode:=cpu max_msgs:=1000
"""

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
                    'communication,backend,size_bytes,resolution,target_hz,frame_cap,'
                    'received_frames,avg_latency_ms,min_latency_ms,max_latency_ms,std_dev_ms\n')
        except OSError:
            pass


def print_csv_results(event, context):
    csv_file = context.launch_configurations.get('csv_path', '')
    if csv_file and os.path.exists(csv_file):
        try:
            with open(csv_file, 'r') as f:
                content = f.read().strip()
            if content:
                mode = context.launch_configurations.get('mode', 'cuda')
                print('\n' + '=' * 88)
                print(f' INTRA-PROCESS BENCHMARK RESULTS (mode={mode}):')
                print('=' * 88)
                print(content)
                print('=' * 88)
                print(f' Results saved to: {csv_file}\n')
        except OSError:
            pass


def generate_launch_description():
    # Launch arguments
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
        description=(
            "Backend mode for intra-process benchmark: "
            "'cuda' (CUDA buffer backend) or 'cpu' (standard CPU buffer)"))
    max_msgs_arg = DeclareLaunchArgument(
        'max_msgs', default_value='1000',
        description='Number of messages to publish/receive (0 = continuous)')
    warmup_msgs_arg = DeclareLaunchArgument(
        'warmup_msgs', default_value='10',
        description='Number of warm-up messages to ignore during setup (default: 10)')
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='/tmp/cuda_intra_process_results.csv',
        description='Path to save output summary CSV results')
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='intra_cuda_image_test',
        description='Topic name (used only internally within the container)')
    history_depth_arg = DeclareLaunchArgument(
        'history_depth', default_value='80',
        description='History depth for buffer pool and QoS queue')
    cuda_device_id_arg = DeclareLaunchArgument(
        'cuda_device_id', default_value='0',
        description='CUDA GPU device ordinal')

    # True when mode is 'cuda'
    use_cuda = PythonExpression([
        "'", LaunchConfiguration('mode'), "'.strip().lower() in ['cuda', 'gpu', 'true', '1']"
    ])

    # auto_exit: always True when max_msgs > 0
    auto_exit_expr = PythonExpression([
        "'true' if int('", LaunchConfiguration('max_msgs'), "') > 0 else 'false'"
    ])

    publisher_node = ComposableNode(
        package='cuda_bb_benchmark',
        plugin='cuda_bb_benchmark::CudaImagePublisherNode',
        name='intra_image_publisher',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'rate': LaunchConfiguration('rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'use_cuda': use_cuda,
            'wait_for_subscriber': True,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    subscriber_node = ComposableNode(
        package='cuda_bb_benchmark',
        plugin='cuda_bb_benchmark::CudaImageSubscriberNode',
        name='intra_image_subscriber',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'result_prefix': 'cuda_intra',
            'use_cuda': use_cuda,
            'require_cuda': use_cuda,
            'verify_content': False,
            'publish_diagnostics': False,
            'history_depth': LaunchConfiguration('history_depth'),
            'max_messages': LaunchConfiguration('max_msgs'),
            'warmup_messages': LaunchConfiguration('warmup_msgs'),
            'target_hz': LaunchConfiguration('rate'),
            'summary_csv_path': LaunchConfiguration('csv_path'),
            'auto_exit': auto_exit_expr,
            'cuda_device_id': LaunchConfiguration('cuda_device_id'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='cuda_intra_process_container',
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
        cuda_device_id_arg,
        OpaqueFunction(function=init_csv_file),
        container,
        exit_event_handler,
        shutdown_event_handler,
    ])
