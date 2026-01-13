# Copyright (C) 2024 Robin Baran
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():


    mast_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="move_mast",
        output='screen'
    )

    wheel_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="move_wheel",
        output='screen'
    )

    run_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="run_demo",
        output='screen'
    )

    odom_node = Node(
        package="lunar_pole_exploration_rover_demo",
        executable="odom_tf_publisher",
        output='screen'
    )


    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        mast_node,
        wheel_node,
        run_node,
        odom_node
    ])
