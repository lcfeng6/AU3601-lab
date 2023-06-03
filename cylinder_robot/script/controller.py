#!/usr/bin/env python
#coding=utf-8

import rospy
import sys
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

#定义基础类PID_controller来进行基本PID操作，包括pid参数初始化，pid输出函数，以及模块化封装的置零操作函数等
class PID_Controller:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    #获取pid输出函数
    def get_output(self, error):
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error
 
        output = self.kp * self.error + \
            self.ki * self.error_sum + \
            self.kd * self.error_diff
        self.output = self.constrain(output)
        return self.output

    #定义置零函数，模块化封装了将pid过程变量置零的操作
    def clear(self):
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        return



#自定义一个类来执行pid操作，包括初始化并获取位置速度变量、pid调节等功能
class myPID():
    #初始化函数，初始化了位置速度变量和发布、订阅模块等
    def __init__(self, pub):
        rospy.Subscriber("/robot/esti_model_state",ModelState,self.callback_control)
        #通过一系列坐标来设置轨迹，通过state变量判断当前处于哪一段轨迹
        self.trace=np.array([[-1,-1], [-2, 0], [-1, 1], [0, 0],\
            [1, -1], [2, 0], [1, 1], [0, 0]])
        self.px=0
        self.py=0
        self.vx=0
        self.vy=0
        self.pub = pub
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(10)
        self.state = 0
        
    #获取位置速度
    def callback_control(self,msg):
        self.px=msg.pose.position.x
        self.py=msg.pose.position.y
        self.vx=msg.twist.linear.x
        self.vy=msg.twist.linear.y

    #双层pid控制：外层通过调节速度控制位置，内层通过调节力控制速度，位置和速度的pid参数和阈值由外部输入，方便调节
    def double_controler(self, p_pid, v_pid, thre):
        position_controller_x = PID_Controller(*p_pid)
        position_controller_y = PID_Controller(*p_pid)
        velocity_controller_x = PID_Controller(*v_pid)
        velocity_controller_y = PID_Controller(*v_pid)

        self.p_thre = thre
        self.v_thre = thre

        self.rate.sleep()
        while not rospy.is_shutdown():
            if self.state == len(self.trace):
                break
            else:
                error_x, error_y = self.trace[self.state][0] - self.px,\
                    self.trace[self.state][1] - self.py
                #如果位置误差小于阈值就进入下一段轨迹，如果有任意一个坐标不合阈值就继续调节
                if abs(error_x) < self.p_thre and abs(error_y) < self.p_thre:
                    self.state += 1
                    position_controller_x.clear()
                    position_controller_y.clear()
                else:
                    vx = position_controller_x.get_output(error_x)
                    vy = position_controller_y.get_output(error_y)
                    error_vx, error_vy = vx - self.vx, vy - self.vy
                    #x与y的速度分别影响两个分量的位移互不干扰，因此分别判断是否达标
                    if abs(error_vx) < self.v_thre:
                        self.cmd_vel.linear.x = 0
                        velocity_controller_x.clear()
                    else:
                        self.cmd_vel.linear.x = velocity_controller_x.get_output(error_vx)

                    if abs(error_vy) < self.v_thre:
                        self.cmd_vel.angular.z = 0
                        velocity_controller_y.clear()
                    else:
                        self.cmd_vel.angular.z = velocity_controller_y.get_output(error_vy)

            self.pub.publish(self.cmd_vel)
            rospy.loginfo("cmd_vel: [%0.2f m/s, %0.2f rad/s] self.state: %0.1f", self.cmd_vel.linear.x, self.cmd_vel.angular.z, self.state)
            rospy.loginfo("error: x[%0.2f , %0.2f ], v[%0.2f , %0.2f] position: [%0.1f,%0.1f] velocity: [%0.2f , %0.2f]",error_x,error_y,error_vx,error_vy,self.px,self.py,self.vx,self.vy)
            self.rate.sleep()

        return






def controller():
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)

    rospy.init_node('controller',anonymous=True)
    #============design your trace below=============
    pid = myPID(pub)
    p_pid = [0.3, 0, 0.0035, -1, 1]
    v_pid = [3, 0, 0.037, -1, 1]
    pid.double_controler(p_pid, v_pid, 1.5e-2)
    sys.exit(0)



if __name__ == '__main__':
    try:
        controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


























