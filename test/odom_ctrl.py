#!/usr/bin/python3
import math
import rclpy
import json
import os

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix,Imu
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist,Point,QuaternionStamped
from rclpy.time import Time

import tf_transformations
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException, Buffer, TransformListener
from basic_math import *


class RobotController(Node):
    '''结合路径信息与 PID 控制器进行运动控制'''
    def __init__(self):
        super().__init__('gps_pid_ctrl_node')

        self.current_index = 0  # 用于记录当前点的索引
        self.gps_path = Path()  # 存储路径点信息
        self.current_pose = Point()
        self.look_ahead_index = 1  # 初始瞄准点索引，设置为比当前点前一点
        self.robot_orientation = None  # 用于存储机器人的方向
        self.robot_odom_orientation = None
        self.robot_odom_combined_orientation = None
        self.robot_heading_orientation = None

        self.max_V=0.40             # 最大速度
        self.max_VTH=1.1            # 最大角速度
        self.v = 0.0

        '''导航模型属性'''
        self.M=0.95                 # 等效为机器人的质量, a=F/M
        self.lookAhead=0.6          # 机器人在导航过程中的前视角
        self.ptn_resolution=0.05    # 机器人在导航过程中导航点集的密集度
        self.cmd_pub_Rate=100        # 机器人速度更新频率
        self.max_V=0.50             # 最大速度
        self.max_Vth=1.0            # 最大角速度
        self.max_Accel_Ang=4        # 机器人的最大角度加速度
        self.max_Accel=1.5          # 这个值代表了两点:1)机器人在最大牵引下的加速度 2)如果没有外力牵引,机器人会在最大速度下以该加速度进行衰减
        self.robot_Radius=0.35      # 机器人的半径

        self.decay_Ratio=self.max_Accel/self.max_V/self.cmd_pub_Rate                #计算出机器人的衰减特性
        self.decay_Ratio_Ang=self.max_Accel_Ang/self.max_Vth/self.cmd_pub_Rate      #机器人的角速度衰减特性
        
        self.max_Force=self.decay_Ratio*self.max_V                                  # 计算最大拉力特性
        self.max_Force_Ang=self.decay_Ratio_Ang*self.max_Vth/self.cmd_pub_Rate      #在角度为180度时,达到最大的拉力

        #
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化机器人位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 初始化
        self.pid_controller = PIDController(kp=1.0, ki=0.1, kd=0.01)  # PID 控制器
        # 参数
        self.declare_parameter('user_config', '/home/robot/turtlebot3_ws')
        user_config_param = self.get_parameter('user_config').get_parameter_value().string_value  # 获取参数
        self.gps_walked_file_path = os.path.join(user_config_param, 'odom_combined_walked_path.json')  # 保存路径
        # 话题
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10) # 发布速度指令
        self.state_pub_ = self.create_publisher(Path, 'walked_path', 10) # 发布路径

        self.odom_combined_subscription = self.create_subscription(
            Odometry,  # 需要导入odom消息类型
            '/odom_combined',
            self.odom_combined_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,  # 需要导入odom消息类型
            '/odom',
            self.odom_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,  # 需要导入IMU消息类型
            '/imu',
            self.imu_callback,
            10
        )

        # 定时器，每0.1秒调用一次navigate_to_target
        self.timer = self.create_timer(0.01, self.navigate_to_target)  # 以0.1秒的频率导航

        # 加载路径数据
        self.load_path_from_json()

    
    def odom_callback(self,msg=Odometry()):
        # print(msg)
        # self.current_pose.x = msg.pose.pose.position.x
        # self.current_pose.y = msg.pose.pose.position.y
        self.robot_odom_orientation = {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }

    def odom_combined_callback(self,msg=Odometry()):
        # print(msg)
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        self.robot_odom_combined_orientation = {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
        # print(self.robot_odom_orientation)
    def imu_callback(self,msg):
        self.robot_orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
    

    def quaternion_to_yaw(self,orientation):

        if orientation is None:
            return 0.0
        x = orientation['x']
        y = orientation['y']
        z = orientation['z']
        w = orientation['w']

        # 将四元数转换为euler欧拉角 (roll, pitch, yaw)
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            [x, y, z, w]
        )
        return yaw

    def update_pose(self):
        try:
            # 查找从 map 到 base_link 的变换
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, now)

            # 提取平移和旋转部分
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # 四元数转欧拉角，获取 yaw
            r, p, y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            # 更新位姿
            self.x = trans.x
            self.y = trans.y
            self.theta = y

        except TransformException as ex:
            self.get_logger().warn(f'[dynamic planner] No transform from {self.map_frame} to {self.base_frame}: {ex}')



    def load_path_from_json(self):
        try:
            """从 JSON 文件中读取路径数据并构建为 ROS Path 类型"""
            with open(self.gps_walked_file_path, 'r') as json_file:
                path_data = json.load(json_file)

            self.gps_path.header.frame_id = "odom_combined"
            self.gps_path.header.stamp = self.get_clock().now().to_msg()  # 路径的时间戳

            # 逐个读取 JSON 中的位置信息并转换为 PoseStamped
            for pose_data in path_data:
                pose = PoseStamped()
                pose.pose.position.x = pose_data["position"]["x"]
                pose.pose.position.y = pose_data["position"]["y"]
                pose.pose.position.z = pose_data["position"]["z"]

                # 将 JSON 中的时间戳转换为 ROS Time 类型
                timestamp = pose_data["timestamp"]
                sec, nanosec = divmod(timestamp, 1)  # 将秒和纳秒分离
                pose.header.stamp = Time(seconds=int(sec), nanoseconds=int(nanosec * 1e9)).to_msg()

                # 添加到路径列表
                self.gps_path.poses.append(pose)
            self.state_pub_.publish(self.gps_path)
            self.get_logger().info(f"Loaded {len(self.gps_path.poses)} points from path")
        except Exception as e:
            self.get_logger().error(f"Failed to load path data: {e}")


    def navigate_to_target(self):
        '''逐步导航到目标点'''
        if self.current_index >= len(self.gps_path.poses):
            twist_msg = Twist()
            twist_msg.linear.x = 0.0 # 设置固定线速度，或根据需求动态调整
            twist_msg.angular.z = 0.0 # 使用PID控制器计算角速度
            
            # 发布速度指令
            self.publisher.publish(twist_msg)
            self.get_logger().info("All points reached")
            return

        # 获取当前点和目标点
        target_pose = self.gps_path.poses[self.current_index].pose.position

        # 瞄准点 - 确保索引不超出路径长度
        look_ahead_index = min(self.current_index + self.look_ahead_index, len(self.gps_path.poses) - 1)
        look_ahead_pose = self.gps_path.poses[look_ahead_index].pose.position

        x_robot = self.current_pose.x
        y_robot = self.current_pose.y
        theta = self.quaternion_to_yaw(self.robot_odom_orientation) if self.robot_odom_orientation else 0.0



        # 计算当前位置与目标点的误差
        error_x = target_pose.x - self.current_pose.x
        error_y = target_pose.y - self.current_pose.y
        distance_to_target = math.sqrt(error_x ** 2 + error_y ** 2)

         # 计算瞄准点的方向
        look_ahead_error_x = look_ahead_pose.x - x_robot
        look_ahead_error_y = look_ahead_pose.y - y_robot
        look_ahead_angle = math.atan2(look_ahead_error_y, look_ahead_error_x)

        # 计算角度差
        angle_diff = normalize_angle(look_ahead_angle - theta)

        # 坐标变换
        # self.update_pose()

        # 从四元数获取当前方向角
        # odom
 

        # current_odom_combined_angle =self.quaternion_to_yaw(self.robot_odom_combined_orientation) if self.robot_odom_combined_orientation else 0.0
        # current_odom_angle =self.quaternion_to_yaw(self.robot_odom_orientation) if self.robot_odom_orientation else 0.0
        # current_imu_angle =self.quaternion_to_yaw(self.robot_orientation) if self.robot_orientation else 0.0
        
        # self.get_logger().info(f"imu Current angle: {math.degrees(current_imu_angle):.2f} degrees")
        # self.get_logger().info(f"odom Current angle: {math.degrees(current_odom_angle):.2f} degrees")
        # self.get_logger().info(f"odom combined Current angle: {math.degrees(current_odom_combined_angle):.2f} degrees")

        #self.theta = 0.0
        # current_angle = self.theta
        # current_angle = current_odom_combined_angle
        # 计算距离目标点的方向角
        # target_angle = normalize_angle(math.atan2(error_y, error_x))

        # 计算角度差
        # angle_diff = normalize_angle(target_angle - current_angle)

        # self.get_logger().info(f"Distance to target: {distance_to_target:.2f} meters")
        # self.get_logger().info(f"Current angle: {math.degrees(current_angle):.2f} degrees")
        # self.get_logger().info(f"Target angle: {math.degrees(target_angle):.2f} degrees")
        # self.get_logger().info(f"Angle difference: {math.degrees(angle_diff):.2f} degrees")

        # 定义阈值
        distance_threshold = 0.2  # 米
        angle_threshold = math.radians(10)  # 弧度，10度


        # 判断是否到达目标点
        if distance_to_target < distance_threshold and abs(angle_diff) < angle_threshold:
            self.get_logger().info(f"Reached target at ({target_pose.x:.2f}, {target_pose.y:.2f})")
            self.current_index += 1  # 更新到下一个目标点
            return

        # 使用 PID 控制器计算转向输出
        direction_output = self.pid_controller.update(look_ahead_angle, theta, 0.01)  # 使用角度差作为输入

        #
        self.vth=Theta2AccelAngluar(angle_diff,self.max_VTH)
      
        self.v = self.smooth_speed(self.v, distance_to_target, 0.5)

        # 调整线速度与角速度的关系
        if abs(angle_diff) > angle_threshold:
            self.v = 0.0
        else:
           
            self.v = distance_to_target

        # if abs(angle_diff) > angle_threshold:
        #     # 角度差较大，优先调整方向，减小线速度
        #     # direction_output = 0.5
          
       
        #     self.v = 0.01
        # else:
        #     # 角度差较小时，根据距离调整线速度
        #     dV = Theta2AccelLinear(angle_diff, self.max_Force)
        #     self.v = (1 - self.decay_Ratio) * self.v + dV * self.decay_Ratio
        
        self.set_robot_speed(self.v, direction_output)
    
    def smooth_speed(self,current_speed, target_speed, max_acceleration):
        if abs(target_speed - current_speed) < max_acceleration:
            return target_speed
        return current_speed + max_acceleration * (1 if target_speed > current_speed else -1)



    def set_robot_speed(self, speed, turn_rate):
        # 将 PID 输出转化为机器人的线速度和角速度
        # 发送速度控制命令
        # 可以是发布一个 Twist 消息或调用其他运动控制方法
        twist_msg = Twist()
        twist_msg.linear.x = speed # 设置固定线速度，或根据需求动态调整
        twist_msg.angular.z = turn_rate # 使用PID控制器计算角速度
        
        # 发布速度指令
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Set speed: {speed:.2f}, Turn rate: {turn_rate:.2f}")


###功能函数###


def normalize_angle(angle):
    """规范化角度，使其保持在 [-pi, pi] 范围内"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PIDController:
    ''' PID 控制器类，用于计算控制输出'''
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        error = normalize_angle(error)  # 确保误差在 [-pi, pi] 范围内
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def main(args=None):
    rclpy.init(args=args)
    path_tracking_node = RobotController()
    rclpy.spin(path_tracking_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
