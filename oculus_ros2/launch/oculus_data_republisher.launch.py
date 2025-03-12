#
# @file - oculus_data_republisher.launch.py
# @author - Ian MCELROY, Forssea Robotics (ian@forssea-robotics.fr)
# @brief - Launch Oculus data republisher
# @version - 0.1
# @date - 2025-03-11
#
# Copyright (c) 2025 Forssea Robotics. All rights reserved.
#

# BSD 3-Clause License
#
# Copyright (c) 2022, ENSTA-Bretagne
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)

def create_oculus_processes(
    context: LaunchContext,
    data_file: LaunchConfiguration,
):
    data_file_substituted = data_file.perform(context)

    processes = []
    
    processes.append(
        Node(
            package="oculus_ros2",
            executable="oculus_republisher_node",
            name="oculus_republisher",
            remappings=[("image", "oculus_sonar/image")],
            parameters=[{
                "data_filename": data_file_substituted,
                "host": "0.0.0.0",
                "port": 12345,
            }],
            output="screen",
        )
    )

    return processes

def generate_launch_description():
    data_file = LaunchConfiguration("data_file")

    launch_list = [
        DeclareLaunchArgument(
            name="data_file",
            description="Oculus data file to republish.",
        ),
        OpaqueFunction(
            function=create_oculus_processes,
            args=[data_file],
        ),
    ]

    return LaunchDescription(launch_list)
