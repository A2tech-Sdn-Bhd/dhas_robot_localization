# Copyright 2019 Samsung Research America
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
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dhas_dual_ekf_navsat.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    rviz_config_file = '/home/leggeed/dhas_dev_ws/src/dhas_nodes/rviz/localization.rviz'
    return LaunchDescription([
        launch_ros.actions.Node(
                package='robot_localization', 
                executable='navsat_transform_node', 
                name='navsat_transform',
                output='screen',
                parameters=[parameters_file_path],
                remappings=[('imu', '/rtk_heading/imu'),
                            ('gps/fix', 'gps/rover/fix'), 
                            ('gps/filtered', 'gps/filtered'),
                            ('odometry/gps', 'odometry/gps'),
                            ('odometry/filtered', 'odometry/global')]           

            ),
        launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d',rviz_config_file],
                parameters=[{'use_sim_time': True}]
            )    
])
