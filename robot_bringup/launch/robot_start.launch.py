import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # 获取机器人的share目录路径
    bringup_dir = get_package_share_directory('robot_bringup')
    config_dir = os.path.join(bringup_dir, 'config')

    # 参数文件路径
   

    #-------------------------#
    
    # 获取仿真机器人的share目录路径
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_gazebo_launch_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')

    # 仿真机器人
    urtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_launch_dir, 'turtlebot3_world.launch.py')),
    )

    # 参数融合节点
    robot_localizaton_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'robot_localization_ekf.launch.py'))
    )

    
    #-------------------------#

    # 创建LaunchDescription对象，并添加电机驱动节点和传感器launch文件的动作
    ld = LaunchDescription()
    ld.add_action(urtlebot3_gazebo)
    ld.add_action(robot_localizaton_ekf_launch)
    
    return ld
