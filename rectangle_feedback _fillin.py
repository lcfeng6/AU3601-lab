#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np
import matplotlib.pyplot as plt 

x, y, theta = 0, 0, 0 # initial
time_now = 0.

class PID_Controller:
    def __init__(self,kp,ki,kd,output_min,output_max):
        # 初始化PID的三个参数，以及误差项
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0 # 误差
        self.last_error = 0 # 上一时刻误差
        self.error_sum = 0 # 误差累加，代替积分
        self.error_diff = 0 # 误差差分，代替微分
        
        # 初始化最大输出与最小输出
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0 # 初始化控制器输出

    def constrain(self, output):
        # 控制器输出阈值限制
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        # 使用位置式PID获取输出
        self.error = error
        self.error_sum += self.error # 误差累加
        self.error_diff = self.error - self.last_error # 误差差分
        self.last_error = self.error

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)

        return self.output

def draw_error(Error, figurename, figurelabel = 1):
    # 绘制误差
    Error[:, 0] -= Error[1, 0]
    traget = np.zeros(shape=(Error.shape[0],))
    plt.figure(figurelabel)
    plt.plot(Error[:, 0], Error[:, 1], label='error', color='blue')
    plt.plot(Error[:, 0], traget, label='target', color='red',linestyle='--')
    plt.xlabel("time(s)")
    plt.ylabel(figurename) 
    plt.legend()

def PoseCallback(pose):
    # 获取小海龟的初始位置信息，以便后计算相对位移
    global x, y, theta
    global time_now
    x = pose.x - 5.544444561  # 5.544444561 是初始
    y = pose.y - 5.544444561
    theta = pose.theta
    get_time = rospy.Time.now() # 获取当前时间
    time_now = get_time.to_sec() # 转化时间格式为秒

def velocity_publisher():
    global x, y, theta
    global time_now
	# ROS节点初始化
    rospy.init_node('velocity_publisher', anonymous=True)
    
    # 新建订阅者subscriber，话题为'/turtle1/pose'，类型为Pose，回调函数为PoseCallback
    rospy.Subscriber('/turtle1/pose', Pose, PoseCallback)
    
    # 新建发布者publisher，话题为'/turtle1/cmd_vel'，类型为Twist，缓存10帧数据
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # 初始化数组，用于保存error
    TransError_x = np.empty(shape=(0, 2)) 
    TransError_y = np.empty(shape=(0, 2))
    RotError_z = np.empty(shape=(0, 2))

	#设置循环的频率
    rate = rospy.Rate(10)
    # 设置小乌龟初始state
    state = 0
    cmd_vel = Twist()
    
    # 填入轨迹形状，若为矩形，则如下所示：
    L, H = ..., ...  # 矩形长宽
    
    # 误差阈值
    move_er_threshold = 1e-4
    rotate_er_threshold = 1e-3
    
    # PID控制器部分实例化，PID参数可调
    # 输入合适的PID参数
    move_controller = PID_Controller(..., ..., ..., ..., ...) 
    rotate_controller = PID_Controller(..., ..., ..., ..., ...) 
    
    rate.sleep() # 等待Subscriber，Publisher初始化
    while not rospy.is_shutdown():
        # mode0: 使得小乌龟前进L距离，画出矩形的一条边
        if state == 0:
            error = L - x # 计算误差
            TransError_x = np.append(TransError_x, [[time_now, error]], axis=0)
            if abs(error) < move_er_threshold: # 误差小于阈值，说明到达位置
                state = state + 1 # 进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0  # 重置控制器
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        # mode1: 使得小乌龟旋转math.pi / 2，画出矩形的直角部分
        elif state == 1:
            error = math.pi / 2 - theta # 计算误差
            RotError_z = np.append(RotError_z, [[time_now, error]], axis=0)
            if abs(error) < rotate_er_threshold: # 误差小于阈值，说明到达位置
                state = state + 1 # 进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rotate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rotate_controller.get_output(error)
        ##===================================================================##
        ##            注意：现存代码仅完成了矩形的一条边与一个角的绘制           ##
        ##           要求写出剩下的部分，使得小乌龟可以画出一个完整的矩形         ##
        ##                                                                   ##
        ##===================================================================##

		# 发布消息
        turtle_vel_pub.publish(cmd_vel)
        rospy.loginfo("cmd_vel: [%0.2f m/s, %0.2f rad/s] state: %0.1f", cmd_vel.linear.x, cmd_vel.angular.z, state)
		# 按照循环频率延时
        rate.sleep()
    
    if (TransError_x.size):
        draw_error(TransError_x, "Trans_x", 1)
    if (TransError_y.size):
        draw_error(TransError_y, "Trans_y", 2)
    if (RotError_z.size):
        draw_error(RotError_z, "Rot_z", 3)
    plt.show()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass