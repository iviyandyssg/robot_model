import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry

#import tf_transformations

import random

#import math

 

class OdometryWithNoise(Node):

    def __init__(self, name: str = "OdometryWithNoise", 
                 sub_topic: str = "odom", 
                 pub_topic: str = "odom_n",
                 p: float = 0.05):
        super().__init__(name)
        self.precent = p
        self.sub_odom = self.create_subscription(
            Odometry,
            sub_topic,
            self.sub_callback,
            10
        )

        self.pub_odom_noise = self.create_publisher(Odometry, pub_topic, 10)

    

    def sub_callback(self, msg):
        odom_noise = Odometry()
        odom_noise.header = msg.header
        odom_noise.child_frame_id = msg.child_frame_id

        

        # 复制姿态数据
        # odom_noise.pose = msg.pose
        # 复制速度数据
        odom_noise.twist = msg.twist
        odom_noise.pose.pose.position.x = 0.0
        odom_noise.pose.pose.position.y = 0.0
        odom_noise.pose.pose.position.z = 0.0
        odom_noise.pose.pose.orientation.x = 0.0
        odom_noise.pose.pose.orientation.y = 0.0
        odom_noise.pose.pose.orientation.z = 0.0
        odom_noise.pose.pose.orientation.w = 1.0

        # 位置噪声处理
        # position = odom_noise.pose.pose.position

        # 高斯噪声：位置
        # position.x += self.gauss_noise_f(0, abs(position.x * self.precent * 0.2))
        # position.y += self.gauss_noise_f(0, abs(position.y * self.precent * 0.2))
        # position.z += self.gauss_noise_f(0, abs(position.z * self.precent * 0.2))

 

        # 姿态噪声处理
        # orientation = odom_noise.pose.pose.orientation
        # roll, pitch, yaw = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        #roll += self.gauss_noise_f(0, abs(roll * self.precent * 0.1))
        #pitch += self.gauss_noise_f(0, abs(pitch * self.precent * 0.1))
        #yaw += self.gauss_noise_f(0, abs(yaw * self.precent * 0.1))
        
        #new_quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        #orientation.x = new_quat[0]
        #orientation.y = new_quat[1]
        #orientation.z = new_quat[2]
        #orientation.w = new_quat[3]

 

        # 速度噪声处理
        twist = odom_noise.twist.twist
        # 均匀分布噪声：线速度
        twist.linear.x += self.random_noise_f(-abs(twist.linear.x * self.precent), abs(twist.linear.x * self.precent))
        twist.linear.y += self.random_noise_f(-abs(twist.linear.y * self.precent), abs(twist.linear.y * self.precent))
        twist.linear.z += self.random_noise_f(-abs(twist.linear.z * self.precent), abs(twist.linear.z * self.precent))
        # 均匀分布噪声：角速度
        twist.angular.x += self.random_noise_f(-abs(twist.angular.x * self.precent), abs(twist.angular.x * self.precent))
        twist.angular.y += self.random_noise_f(-abs(twist.angular.y * self.precent), abs(twist.angular.y * self.precent))
        twist.angular.z += self.random_noise_f(-abs(twist.angular.z * self.precent), abs(twist.angular.z * self.precent))

        self.pub_odom_noise.publish(odom_noise)

 

    def gauss_noise_f(self, mu: float, sigma: float) -> float:

        return random.gauss(mu, sigma)

    

    def random_noise_f(self, min_val: float, max_val: float) -> float:

        return random.uniform(min_val, max_val)

 

def main(args=None):

    rclpy.init(args=args)

    node = OdometryWithNoise()

    rclpy.spin(node)

    rclpy.shutdown()

 

if __name__ == '__main__':

    main()