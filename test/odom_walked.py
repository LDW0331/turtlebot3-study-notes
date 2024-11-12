#!/usr/bin/python3
import os
import json
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,Odometry


class OdomPosition:
    """用于存储odom位置的类"""
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y




class OdomPath(Node):
    def __init__(self):
        super().__init__('odom_path_node')
        self.pose_init = False
        self.init_pose = None  # 初始位置
        self.last_recorded_pose = None  # 用于记录上一次记录的位置(GPS坐标)
        self.position_threshold = 0.05  # 位置变化的阈值，单位：米
        self.ros_path_ = Path()

        # 声明参数
        self.declare_parameter('user_config', '/home/robot/turtlebot3_ws')
        # 获取参数
        user_config_param = self.get_parameter('user_config').get_parameter_value().string_value
        self.get_logger().info(f"user_config_param = {user_config_param}")

        # 保存路径
        self.walked_file_path = os.path.join(user_config_param, 'walked_path.json')
        self.state_pub_ = self.create_publisher(Path, 'gps_path', 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.odom_combined_callback,
            10
        )
        self.ros_path = Path()
        self.ros_path.header.frame_id = 'odom_path'

    def odom_combined_callback(self, msg=Odometry()):

        if not self.pose_init:
            # 初始化初始位置
            
            self.init_pose = OdomPosition(msg.pose.pose.position.x,msg.pose.pose.position.y)
            self.last_recorded_pose = self.init_pose  # 初始化时也设置最后记录位置

            self.pose_init = True
            self.get_logger().info("Initialized odom Position")
        else:
            # 检查位置是否发生显著变化
            if self.has_position_changed(msg.pose.pose.position.x,msg.pose.pose.position.y):
                # 创建PoseStamped对象并添加时间戳
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
         
                pose.pose.position.x = msg.pose.pose.position.x
                pose.pose.position.y = msg.pose.pose.position.y
                pose.pose.position.z = msg.pose.pose.position.z
                
                self.ros_path.poses.append(pose)



                # 发布路径
                # ros_path_ = Path()
                self.ros_path_.header.frame_id = 'odom_combined'
                self.ros_path_.header.stamp = self.get_clock().now().to_msg()
                pose = PoseStamped()
                pose.header = self.ros_path_.header
                pose.pose.position.x = msg.pose.pose.position.x
                pose.pose.position.y = msg.pose.pose.position.y
                pose.pose.position.z = msg.pose.pose.position.z
                self.ros_path_.poses.append(pose)
                
                self.state_pub_.publish(self.ros_path_)

                # 更新最后记录的位置
                self.last_recorded_pose = OdomPosition(msg.pose.pose.position.x,msg.pose.pose.position.y)

                # 保存路径到 JSON 文件
                self.save_path_to_json(self.walked_file_path)

                self.get_logger().info(f"#Published odom : x={msg.pose.pose.position.x:.6f}, y={msg.pose.pose.position.y:.6f}")

    def has_position_changed(self,x,y):
        """判断当前位置是否与上一次记录的位置有显著变化"""

        # UMT 平面坐标计算
        error_x = x - self.last_recorded_pose.x
        error_y = y - self.last_recorded_pose.y
        distance = math.sqrt(error_x ** 2 + error_y ** 2)

        distance = round(distance,2)

    
        if distance >= self.position_threshold:
            self.get_logger().warning(f"distance:{distance}")
            return True
        else:
            return False
   
    def save_path_to_json(self,path):
        """将路径数据写入JSON文件"""
        path_data = []
        for pose in self.ros_path.poses:
            pose_data = {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                },
                "timestamp": pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            }
            path_data.append(pose_data)

        # 将路径数据写入JSON文件
        with open(path, 'w') as json_file:
            json.dump(path_data, json_file, indent=4)


def main(args=None):
    rclpy.init(args=args)
    path = OdomPath()

    try:
        rclpy.spin(path)
    except KeyboardInterrupt:
        path.get_logger().info('Shutting down GPS Path node...')
    finally:
        path.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
