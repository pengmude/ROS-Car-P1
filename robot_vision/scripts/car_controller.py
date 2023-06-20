#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8, Int32
from geometry_msgs.msg import Twist


class RosCarController:

    
    def __init__(self):

        # init ROS node 
        rospy.init_node("car_controller",anonymous=True)
        rospy.loginfo("Satar car node……")

        # 获取launch参数
        # self.limit_time = float(rospy.get_param('~limit_time',5.0))
        self.v0 = int(rospy.get_param('~v0',40.0))
        if self.v0 < 30 or self.v0 > 120:
            rospy.logerr('Invalid input for v0 variable. Please enter a value within the requred range of [30, 120]!')
            return
        
        self.light_color = 'green'
        self.cur_speed = self.v0
        self.line_pos = -1
        self.image_center = 320
        self.cur_angular = 0.0
        self.limit_flag = False
        self.limit_time = 7.0

        # 订阅红绿灯节点发布的消息
        self.light_sub = rospy.Subscriber("/light_color", String, self.call_back)
        # 订阅识别线路节点的消息
        self.line_sub = rospy.Subscriber("/line_pos", Int32, self.call_back)
        # 订阅限速节点发布的消息
        self.limit_sub = rospy.Subscriber("/max_speed", Int8, self.call_back)
        # 发布小车运动
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.spin()


    def call_back(self, data):

        # 订阅消息得到的红绿灯颜色
        if isinstance(data, String):
            self.light_color = data.data
            # print("traffic_light: {}".format(self.light_color))
        
        # 订阅消息得到的中线位置
        elif isinstance(data, Int32):
            self.line_pos = data.data
            # print("line_pos: {}".format(self.line_pos))
        
        # 订阅消息得到的限速牌的值：-1表示没有限速牌
        elif isinstance(data, Int8):
            limit_speed = data.data
            if not self.limit_flag: # 限速标志没有激活时
                if limit_speed != -1 and self.v0 > limit_speed:
                    # 当有限速牌且初速度v0比限速速度limit_speed大时,要限制速度,激活限速标志
                    self.limit_flag = True
                    self.start_t = time.time()
                    self.cur_speed = limit_speed
                    # 设置小车限速时间，速度越快，限时越短
                    # print("limit_time: {}".format(self.limit_time))
                    if limit_speed == 30:
                        self.limit_time = 10.0
                    elif limit_speed == 40:
                        self.limit_time = 9.5
                    elif limit_speed == 50:
                        self.limit_time = 9.0
                    elif limit_speed == 60:
                        self.limit_time = 8.5
                    elif limit_speed == 70:
                        self.limit_time = 8.0
                    elif limit_speed == 80:
                        self.limit_time = 7.5
                    elif limit_speed == 90:
                        self.limit_time = 7.0
                    elif limit_speed == 100:
                        self.limit_time = 6.5
                    elif limit_speed == 110:
                        self.limit_time = 6.0
                    elif limit_speed == 120:
                        self.limit_time = 5.5
                    else:
                        pass
                else:
                    self.cur_speed = self.v0
                
            else:   # 激活限速标志时
                delta_t = time.time() - self.start_t
                if delta_t > self.limit_time:
                    self.cur_speed = self.v0
                    self.limit_flag = False
                else:
                    # 如果限速没有超过self.limit_time，则不需要改self.cur_speed，继续以原来的速度
                    pass
                
            print("speed: {}".format(self.cur_speed))

        else:
            pass
        
        self.start()


    # 根据线的位置和图像中垂线计算角速度
    def calculate_turning_angle(self, image_center, line_center):
        if float(line_center)/float(image_center) > 0.94 and float(line_center)/float(image_center) < 1.06:
            
            self.cur_angular = 0.0    # 保持直线行驶
        else:
            self.set_angular(image_center, line_center)    # 设置角速度拐弯
        # print('angular: {}'.format(self.cur_angular))

    # 设置角度
    def set_angular(self,image_center,line_center):
        # print('self.image_center: {}, line_center: {}'.format(self.image_center, line_center))
        if self.cur_speed > 20 and self.cur_speed <= 30:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 2.8
        elif self.cur_speed > 30 and self.cur_speed <= 40:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 2.4
        elif self.cur_speed > 40 and self.cur_speed <= 50:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 2.0
        elif self.cur_speed > 50 and self.cur_speed <= 60:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 1.7
        elif self.cur_speed > 60 and self.cur_speed <= 70:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 1.3
        elif self.cur_speed > 70 and self.cur_speed <= 80:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 1.0
        elif self.cur_speed > 80 and self.cur_speed <= 90:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 0.8
        elif self.cur_speed > 90 and self.cur_speed <= 100:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 0.8
        elif self.cur_speed > 100 and self.cur_speed <= 110:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 0.8
        elif self.cur_speed > 110 and self.cur_speed <= 120:
            self.cur_angular = (float(image_center) - float(line_center)) / float(image_center) / 0.8
        else:
            self.cur_angular = 0
        
        
    # 发布运动命令
    def run(self, velocity, angle):
        move_cmd = Twist()
        move_cmd.linear.x = float(velocity)/float(200)
        move_cmd.linear.y = 0
        move_cmd.linear.z = 0
        move_cmd.angular.x = 0
        move_cmd.angular.y = 0
        move_cmd.angular.z = angle
        self.pub_cmd.publish(move_cmd)


    # 小车的运动控制
    def start(self):
        # 先判断红绿灯颜色
        if self.light_color == 'green':
            # print('0000')
            # 再判断中线位置
            if self.line_pos == -1:
                # print('1111')
                self.run(0, 0)
            else:
                # 计算角速度
                self.calculate_turning_angle(self.image_center, self.line_pos)
                self.run(self.cur_speed, self.cur_angular)
        else:
            
            self.run(0, 0)


if __name__ == '__main__':

    car_mannager = RosCarController()