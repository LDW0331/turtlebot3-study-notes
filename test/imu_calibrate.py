#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time
import json

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.imu_data_raw = Imu()
        self.imu_bias = Imu()  # 偏置存储变量
        self.imu_samples = []
        self.last_sample_time = 0.0  # 上一个采样时间
        self.sample_interval = 0.1  # 设定的采样间隔，单位为秒

        self.imu_bias_path = "/home/robot/turtlebot3_ws/imu_bias.json"
 
        # 获取IMU数据
        self.imu_subscription = self.create_subscription(
            Imu,  # 需要导入IMU消息类型
            '/imu',
            self.imu_raw_callback,
            10
        )
        

    def imu_raw_callback(self,msg = Imu()):
        self.imu_data_raw = msg
        current_time = self.get_clock().now().seconds_nanoseconds()[0]  # 获取当前时间戳（秒）
        
        # 判断是否达到采样间隔
        if current_time - self.last_sample_time >= self.sample_interval:
            self.sample_data()
            self.last_sample_time = current_time  # 更新上一个采样时间

    def sample_data(self):
        
        imu_data = self.imu_data_raw
        self.imu_samples.append([
            imu_data.angular_velocity.x,
            imu_data.angular_velocity.y,
            imu_data.angular_velocity.z,
            imu_data.linear_acceleration.x,
            imu_data.linear_acceleration.y,
            imu_data.linear_acceleration.z
        ])

        # self.get_logger().info(f"采集到样本: {imu_data}")

        if len(self.imu_samples) >= 50:  # 100 个样本
            self.calculate_bias()

            # 输出偏置
            self.get_logger().info(f"计算得到的偏置: "
                                    f"[{self.imu_bias.angular_velocity.x}, "
                                    f"{self.imu_bias.angular_velocity.y}, "
                                    f"{self.imu_bias.angular_velocity.z}],"
                                    f"[{self.imu_bias.linear_acceleration.x},"
                                    f"{self.imu_bias.linear_acceleration.y},"
                                    f"{self.imu_bias.linear_acceleration.z}]")
            
            imu_bias_temp= {
                "imu_bias":{
                    "angular_velocity.x":self.imu_bias.angular_velocity.x,
                    "angular_velocity.y":self.imu_bias.angular_velocity.y,
                    "angular_velocity.z":self.imu_bias.angular_velocity.z,
                    "linear_acceleration.x":self.imu_bias.linear_acceleration.x,
                    "linear_acceleration.y":self.imu_bias.linear_acceleration.y,
                    "linear_acceleration.z":self.imu_bias.linear_acceleration.z
                    }
            }
            # 将路径数据写入JSON文件
            with open(self.imu_bias_path, 'w') as json_file:
                json.dump(imu_bias_temp, json_file, indent=4)

    def calculate_bias(self):
        if len(self.imu_samples) == 0:
            self.get_logger().error("没有可用样本进行偏置计算！")
            return
        # 将IMU样本转换为numpy数组以便于计算
        samples_array = np.array(self.imu_samples)

        # 计算陀螺仪偏置
        self.imu_bias.angular_velocity.x = np.mean(samples_array[:, 0])
        self.imu_bias.angular_velocity.y = np.mean(samples_array[:, 1])
        self.imu_bias.angular_velocity.z = np.mean(samples_array[:, 2])

        # 计算加速度计的偏置
        self.imu_bias.linear_acceleration.x = np.mean(samples_array[:, 3])
        self.imu_bias.linear_acceleration.y = np.mean(samples_array[:, 4])
        self.imu_bias.linear_acceleration.z = np.mean(samples_array[:, 5])

        # 针对加速度计进行重力校正
        if self.imu_bias.linear_acceleration.z != 0:
            gravity = 9.81
            scale_factor = gravity / self.imu_bias.linear_acceleration.z
            self.imu_bias.linear_acceleration.x *= scale_factor
            self.imu_bias.linear_acceleration.y *= scale_factor
            self.imu_bias.linear_acceleration.z *= scale_factor
        else:
            self.get_logger().warn("加速度计Z轴偏置为零，无法进行重力校正！")


def main(args=None):
    rclpy.init(args=args)
    imu_calibrator = IMUCalibrator()
    imu_calibrator.get_logger().info("开始IMU偏置校正，静止不动...")
    rclpy.spin(imu_calibrator)
    imu_calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
