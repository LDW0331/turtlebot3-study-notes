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

        '''基本运动属性'''
        self.x=0.0      # 机器人当前位置x
        self.y=0.0      # 机器人当前位置y
        self.theta=0.0  # 机器人当前角度theta
        self.v=0.0      # 机器人当前速度v
        self.vth=0.0    # 机器人当前角速度vth

        '''导航模型属性'''
        self.M=0.95                 # 等效为机器人的质量, a=F/M
        self.lookAhead=0.6          # 机器人在导航过程中的前视角
        self.ptn_resolution=0.05    # 机器人在导航过程中导航点集的密集度
        self.cmd_pub_Rate=15        # 机器人速度更新频率
        self.max_V=0.50             # 最大速度
        self.max_Vth=1.0            # 最大角速度
        self.max_Accel_Ang=4        # 机器人的最大角度加速度
        self.max_Accel=1.5          # 这个值代表了两点:1)机器人在最大牵引下的加速度 2)如果没有外力牵引,机器人会在最大速度下以该加速度进行衰减
        self.robot_Radius=0.35      # 机器人的半径

        ''' 机器人受力特征'''
        self.f1=Vector2D()
        self.f2=Vector2D()
        self.f3=Vector2D()

        self.globalVec=ArrowPoint2D(x1=0.0,y1=0.0,th=0.0)   # 全局坐标系下的向量
        self.localVec=ArrowPoint2D(x1=0.0,y1=0.0,th=0.0)    # 局部坐标系下的向量

        self.decay_Ratio=self.max_Accel/self.max_V/self.cmd_pub_Rate                #计算出机器人的衰减特性
        self.decay_Ratio_Ang=self.max_Accel_Ang/self.max_Vth/self.cmd_pub_Rate      #机器人的角速度衰减特性
        
        self.max_Force=self.decay_Ratio*self.max_V                                  # 计算最大拉力特性
        self.max_Force_Ang=self.decay_Ratio_Ang*self.max_Vth/self.cmd_pub_Rate      #在角度为180度时,达到最大的拉力


        self.totalPtnTrans=0    #   转移路线长度
        self.totalPtnWorking=0  #   工作路线长度
        self.currentPtnTrans=0  #   转移路线的路径点(当前第几个点)
        self.currentPtnWork=0   #   导航路线的路径点(当前第几个点)
        self.lookAhead_Ptns=0   #   瞄准点 像素层面
        self.localPtns=0        #   本地路径数量
        self.targetPtnTrans=0   #   瞄准点ID(转移)
        self.targetPtnWorking=0 #   瞄准点ID(工作中)

        self.current_index = 0      # 用于记录当前点的索引
        self.walked_path = Path()   # 存储路径点信息
        self.pathList=[]            # 存储路径点信息
        self.localPathList=[]       # 存储局部路径点信息
        self.current_pose = Point()

     

       
        self.declare_parameter('user_config', '/home/robot/turtlebot3_ws')
        user_config_param = self.get_parameter('user_config').get_parameter_value().string_value    # 获取参数
        self.walked_file_path = os.path.join(user_config_param, 'walked_path.json')                 # 保存路径
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)                              # 发布速度指令
        self.walked_path_pub = self.create_publisher(Path, 'walked_path', 10)                       # 发布路径


        self.current_xyz_sub = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.current_xyz_callback,10
        )
        
        self.load_path_from_json() # 加载路径数据
        self.timer = self.create_timer(0.01, self.navigate_to_target)  # 定时器，每0.1秒调用一次navigate_to_target # 以0.1秒的频率导航



    
    def current_xyz_callback(self,msg):
        '''订阅当前的GPS 转化的 XYZ'''

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, 
             msg.pose.pose.orientation.w]
        )
        self.theta = yaw
        
    
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


    def load_path_from_json(self):
        try:
            """从 JSON 文件中读取路径数据并构建为 ROS Path 类型"""
            with open(self.walked_file_path, 'r') as json_file:
                path_data = json.load(json_file)

            self.walked_path.header.frame_id = "odom_combined"
            self.walked_path.header.stamp = self.get_clock().now().to_msg()  # 路径的时间戳

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

                # 添加到路径列表 用于发布路径
                self.walked_path.poses.append(pose)
                # 添加转移路径路线
                self.pathList.append(Point2D(pose_data["position"]["x"],pose_data["position"]["y"]))

            # 初始化导航变量
            self.currentPtnTrans=0
            self.totalPtnTrans=len(self.pathList)   # 转移过程的路径点数量
            self.lookAhead_Ptns=2 #瞄准点与当前点之间的点数 12个点

            self.walked_path_pub.publish(self.walked_path)

            
            self.get_logger().info(f"Loaded {len(self.walked_path.poses)} points from path")
        except Exception as e:
            self.get_logger().error(f"Failed to load path data: {e}")


    def navigate_to_target(self):
        '''逐步导航到目标点'''

        if self.currentPtnTrans>=(self.totalPtnTrans-self.lookAhead_Ptns):
            self.set_robot_speed(0.0, 0.0)
            self.get_logger().info("All points reached")
            return
        '''
            开始执行导航过程
            导航过程为逐点导航
            currentPtnTrans: 当前的导航点     totalPtnTrans:转移路径整体导航点
        '''
        # 更新自身位置
        # 构建机器人的自身向量
        # localVec 是一个从机器人当前位置出发，沿着角度 self.theta 方向的单位向量
        self.localVec = ArrowPoint2D(self.x, self.y, self.theta)

        '''
            检测机器人是否应当前进一个点,检测逻辑:
            如果机器人-currentPtns所组成的向量与当前路线向量之间的夹角大于90度(点乘<0)
            证明机器人已经滑过该点
        '''
        # 当前目标点
        current_target = self.pathList[self.currentPtnTrans]
        # 机器人到当前目标点的向量
        robot_to_target_vec = Vector2D(self.x, self.y, current_target.x, current_target.y)
        # 下一路径点向量
        if self.currentPtnTrans + 1 < len(self.pathList):
            next_target = self.pathList[self.currentPtnTrans + 1]
            target_to_next_vec = Vector2D(current_target.x, current_target.y, next_target.x, next_target.y)
        else:
            target_to_next_vec = None

        # 判断是否越过当前目标点
        # if robot_to_target_vec * target_to_next_vec <= 0:
        #     self.currentPtnTrans += 1
        distance_to_current_point = robot_to_target_vec.distance_to(self.localVec)
        print(f"Distance to current point: {distance_to_current_point:.2f}")
        self.switch_threshold = 0.3
        # 判断是否应切换到下一个点：距离接近且机器人位于当前点后方
        if distance_to_current_point < self.switch_threshold and robot_to_target_vec * target_to_next_vec <= 0:
            self.currentPtnTrans += 1
            print(f"Switching to point {self.currentPtnTrans}")
        
        # 瞄准点的索引并获取瞄准点位置
        look_ahead_index = min(self.currentPtnTrans + self.lookAhead_Ptns, self.totalPtnTrans - 1)
        look_ahead_target = self.pathList[look_ahead_index]

        # 机器人到瞄准点的向量
        #robo_to_lookahead_vec = Vector2D(self.x, self.y, look_ahead_target.x, look_ahead_target.y)
        robo_to_lookahead_vec = robot_to_target_vec
        # 归一化并设定最大驱动力
        robo_to_lookahead_vec.Formalize(val=self.max_Force)

        # 综合驱动力向量
        self.f1 = robo_to_lookahead_vec
        self.f3 = robo_to_lookahead_vec + self.f2

        # 计算最终运动角度并调整速度
        theta_diff = self.localVec.Theta(self.f3)
        print(theta_diff)
        self.get_logger().info(f"Angle difference: {math.degrees(theta_diff):.2f} degrees")
        dV = Theta2AccelLinear(theta_diff, self.max_Force)
        self.v = (1 - self.decay_Ratio) * self.v + dV * self.decay_Ratio
        self.vth = Theta2AccelAngluar(theta_diff, self.max_Vth)

        # 发布速度
        self.set_robot_speed(self.v, self.vth)
        




    def set_robot_speed(self, speed, turn_rate):
        # 将 PID 输出转化为机器人的线速度和角速度
        # 发送速度控制命令
        # 可以是发布一个 Twist 消息或调用其他运动控制方法
        twist_msg = Twist()
        twist_msg.linear.x = speed # 设置固定线速度，或根据需求动态调整
        twist_msg.angular.z = turn_rate # 使用PID控制器计算角速度
        
        # 发布速度指令
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Set speed: {speed:.2f}, Turn rate: {turn_rate:.2f}")



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
