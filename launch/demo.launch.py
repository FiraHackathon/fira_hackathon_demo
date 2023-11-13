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
    get_debug_directory,
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
    LogInfo,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_mobile_base_bringup import MobileBaseMetaDescription


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    record = LaunchConfiguration("record").perform(context)
    path_file = 'fira_hackathon_01.traj'

    mobile_base = MobileBaseMetaDescription(f"{demo_config_directory}/robot/base.yaml")
    robot_namespace = mobile_base.get_type()
    demo = "tiara_demo"
    demo_timestamp = get_demo_timestamp()

    self_directory = get_package_share_directory("fira_hackathon_demo")
    debug_directory = get_debug_directory(demo, demo_timestamp, record)
    log_directory = get_log_directory(demo, demo_timestamp, record)

    actions = [
        LogInfo(msg=f"demo_config_directory: {demo_config_directory}"),
        LogInfo(msg=f"debug_directory: {debug_directory}"),
        LogInfo(msg=f"log_directory: {log_directory}"),
    ]

    # in rolling : use launch_ros/launch_ros/actions/set_ros_log_dir.py instead
    actions.append(SetEnvironmentVariable("ROS_LOG_DIR", log_directory))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("tirrex_demo") + "/launch/demo.launch.py"),
            launch_arguments=[
                ("demo", demo),
                ("demo_timestamp", demo_timestamp),
                ("demo_config_directory", demo_config_directory),
                ("mode", mode),
                ("record", record),
                ("robot_namespace", robot_namespace),
            ],
        ))

    path_matching_launch = get_package_share_directory('romea_path_matching_bringup')
    path_matching_launch += '/launch/path_matching.launch.py'
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_matching_launch),
            launch_arguments=[
                ('path_file', path_file),
                ('path_directory', f"{demo_config_directory}/paths"),
                ('robot_namespace', robot_namespace),
                ('configuration_file', f"{demo_config_directory}/path_matching.yaml"),
            ],
        ))

    path_following_launch = get_package_share_directory('romea_path_following_bringup')
    path_following_launch += '/launch/path_following.launch.py'
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_following_launch),
            launch_arguments=[
                ('robot_type', mobile_base.get_type()),
                ('robot_model', mobile_base.get_model()),
                ('robot_namespace', robot_namespace),
                ('configuration_file', f"{demo_config_directory}/path_following.yaml"),
            ],
        ))

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=f"-d {self_directory}/rviz/demo.rviz",
            output={'stdout': 'log'},
        ))

    if record == "true":
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
            choices=['simulation', 'live', 'replay_simulation', 'replay_live'],
        ),
        DeclareLaunchArgument(
            "demo_config_directory",
            default_value=get_package_share_directory("fira_hackathon_demo") + "/config",
        ),
        DeclareLaunchArgument("record", default_value="false"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
