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
echo " Building cuda_bb_benchmark and its dependencies"
echo "=========================================================="

# Source ROS 2 underlay if not already sourced
if [ -z "${ROS_DISTRO}" ]; then
  if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source "/opt/ros/lyrical/setup.bash"
  elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    source "/opt/ros/rolling/setup.bash"
  else
    echo "Error: No ROS 2 underlay found. Please source /opt/ros/<distro>/setup.bash."
    exit 1
  fi
fi

colcon build --packages-up-to cuda_bb_benchmark --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "=========================================================="
echo " Build completed! Source the overlay using:"
echo "   source ${WS_DIR}/install/setup.bash"
echo "=========================================================="
