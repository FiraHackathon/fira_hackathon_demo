# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

from tirrex_demo import (
    get_log_directory,
    get_demo_timestamp,
    save_replay_configuration,
)
from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


LOCAL_CONFIG_DIR = '/cfg_chal3'
DEMO_NAME = 'challenge3'


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    record = LaunchConfiguration("record").perform(context)

    robot_namespace = 'robot'
    demo = DEMO_NAME
    demo_timestamp = get_demo_timestamp()
    self_directory = get_package_share_directory("fira_hackathon_demo")
    log_directory = get_log_directory(demo, demo_timestamp, record)
    demo_config_directory = self_directory + LOCAL_CONFIG_DIR
    tirrex_launch_dir = get_package_share_directory("tirrex_demo") + '/launch'

    actions = [
        SetEnvironmentVariable("ROS_LOG_DIR", log_directory),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tirrex_launch_dir + "/simulator.launch.py"),
            launch_arguments={
                "simulator_type": "gazebo",
                "demo_config_directory": demo_config_directory,
            }.items(),
        ),

        # include the main robot controlled by the user
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(self_directory + '/launch/robot.launch.py'),
            launch_arguments={
                'mode': mode,
                'robot_namespace': robot_namespace,
                'demo_config_directory': demo_config_directory,
            }.items(),
        ),

        # include other robots
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(self_directory + '/launch/robot.launch.py'),
        #     launch_arguments={
        #         'mode': mode,
        #         'robot_namespace': 'vehicle0',
        #         'demo_config_directory': demo_config_directory,
        #     }.items(),
        # ),

        Node(
            package="rqt_runtime_monitor",
            executable="rqt_runtime_monitor",
            name="monitor",
            arguments=['--force-discover'],  # added to fix a bug in ros humble docker
            output={'both': 'log'},  # disable stdout and stderr output on screen
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=['-d', f"{self_directory}/rviz/demo.rviz"],
            output={'stdout': 'log'},
        )
    ]

    if record == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_demo") + "/launch/record.launch.py"),
                launch_arguments={
                    "demo": demo,
                    "demo_timestamp": demo_timestamp,
                    "demo_config_directory": demo_config_directory,
                    "mode": mode,
                    "robot_namespace": robot_namespace,
                }.items(),
            ))

        save_replay_configuration(
            demo,
            demo_timestamp,
            "demo.launch.py",
            {"mode": "replay_" + mode},
        )

    return [GroupAction(actions)]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="simulation",
            choices=['simulation', 'replay_simulation'],
        ),
        DeclareLaunchArgument("record", default_value="false"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
