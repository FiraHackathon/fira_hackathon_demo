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

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_mobile_base_bringup import MobileBaseMetaDescription


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    path_file = LaunchConfiguration("path").perform(context)

    mobile_base = MobileBaseMetaDescription(f"{demo_config_directory}/robot/base.yaml")

    tirrex_launch_dir = get_package_share_directory("tirrex_demo") + '/launch'
    tirrex_robot_dir = tirrex_launch_dir + '/robot'

    path_matching_launch = get_package_share_directory('romea_path_matching_bringup')
    path_matching_launch += '/launch/path_matching.launch.py'

    path_following_launch = get_package_share_directory('romea_path_following_bringup')
    path_following_launch += '/launch/path_following.launch.py'

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tirrex_robot_dir + "/robot.launch.py"),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "robot_configuration_directory": demo_config_directory + "/robot",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tirrex_robot_dir + "/robot_localisation.launch.py"),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "demo_config_directory": demo_config_directory,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_matching_launch),
            launch_arguments=[
                ('path_file', path_file),
                ('path_directory', f"{demo_config_directory}/paths"),
                ('robot_namespace', robot_namespace),
                ('configuration_file', f"{demo_config_directory}/path_matching.yaml"),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_following_launch),
            launch_arguments=[
                ('robot_type', mobile_base.get_type()),
                ('robot_model', mobile_base.get_model()),
                ('robot_namespace', robot_namespace),
                ('configuration_file', f"{demo_config_directory}/path_following.yaml"),
            ],
        )
    ]

    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode"),
        DeclareLaunchArgument("robot_namespace"),
        DeclareLaunchArgument("path"),
        DeclareLaunchArgument("demo_config_directory"),
        OpaqueFunction(function=launch_setup),
    ])
