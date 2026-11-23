#!/usr/bin/env bash
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

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WS_DIR}"

echo "=========================================================="
echo " Installing dependencies for cuda_bb_benchmark workspace"
echo "=========================================================="

# Source ROS 2 underlay if not already sourced
if [ -z "${ROS_DISTRO}" ]; then
  if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source "/opt/ros/lyrical/setup.bash"
  elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    source "/opt/ros/rolling/setup.bash"
  else
    echo "Warning: No ROS 2 underlay found in /opt/ros/{lyrical,rolling}."
  fi
fi

# Ensure rosdep is initialized
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
  echo "Initializing rosdep..."
  sudo rosdep init || true
fi

echo "Updating rosdep database..."
rosdep update --include-eol-distros 2>/dev/null || rosdep update || true

# Target packages up to cuda_bb_benchmark to avoid installing unused deps (e.g. torch, libtorch)
TARGET_PATHS=$(colcon list --packages-up-to cuda_bb_benchmark --paths-only 2>/dev/null || echo "src/cuda_bb_benchmark src/rosidl_buffer_backends/cuda_buffer_backend")

echo "Installing package dependencies via rosdep..."
rosdep install --from-paths ${TARGET_PATHS} --ignore-src -y -r

echo "=========================================================="
echo " Dependencies installed successfully for cuda workspace!"
echo "=========================================================="
