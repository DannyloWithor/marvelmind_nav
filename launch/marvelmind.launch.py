# Copyright 2020 Giovani Bernardes
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

"""Launch Gazebo with a world that has Scooby, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    marvelmind_1 = Node(
        package='marvelmind_nav',
        executable='hedge_rcv_bin',
        name='marvelmind_node',
        output='screen',
        arguments=['/dev/ttyACM1', 'marvel1/'])

    marvelmind_2 = Node(
        package='marvelmind_nav',
        executable='hedge_rcv_bin',
        name='marvelmind_node',
        output='screen',
        arguments=['/dev/ttyACM1', 'marvel2/'])

    return LaunchDescription([
       
        marvelmind_1,
        marvelmind_2
    ])
