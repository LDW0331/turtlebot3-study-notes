# Copyright 2018 Open Source Robotics Foundation, Inc.
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
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成launch文件描述，启动文件配置了两个 EKF 滤波器和一个 navsat_transform_node，
    分别用于局部和全局 odometry 的计算，并将 IMU 和 GPS 数据融合到滤波器中
    Args:
        无
    
    Returns:
        LaunchDescription: launch描述文件，包含节点声明、参数配置等信息
    
    """
    robot_localization_dir = get_package_share_directory('robot_bringup')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'ekf_navsat.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'), # 是否输出最终的位置信息
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'), # 输出位置信息的路径
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path,{'use_sim_time':use_sim_time}],
            remappings=[('odometry/filtered', 'odom_combined')],  #  
            # launch_arguments={'use_sim_time': use_sim_time}.items()        
           ),   # 用于计算局部 odometry/local 的 EKF 滤波器
                # 节点会将轮式里程计和IMU进行融合，输出名为：odometry/local局部里程计
    # launch_ros.actions.Node(
    #         package='robot_localization', 
    #         executable='ekf_node', 
    #         name='ekf_filter_node_map',
	#         output='screen',
    #         parameters=[parameters_file_path],
    #         remappings=[('odometry/filtered', 'odometry/global')]
    #        ),  # 用于计算全局 odometry/global 的 EKF 滤波器
    #            # 将轮式里程计，IMU，和GPS里程计融合，输出 名为：odometry/filtered_map的全局里程计
    # launch_ros.actions.Node(
    #         package='robot_localization', 
    #         executable='navsat_transform_node', 
    #         name='navsat_transform',
	#         output='screen',
    #         parameters=[parameters_file_path],
    #         remappings=[('imu', 'imu/data_raw'), # IMU 数据话题 sub
    #                     ('gps/fix', 'fix'),   # GPS 数据话题 sub
    #                     ('gps/filtered', 'gps/filtered'), # 过滤后的 GPS 数据 pub
    #                     ('odometry/gps', 'odometry/gps'), # 由 GPS 计算的里程计数据 pub
    #                     ('odometry/filtered', 'odom_combined')] # 最终的全局里程计数据，作为全局 EKF 节点的输入    sub   

    #        ),  # 将 IMU 和 GPS 数据融合到滤波器中
])
