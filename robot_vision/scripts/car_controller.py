#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist


class RosCarController:

    
    def __init__(self):
        # init ROS node 
        rospy.init_node("car_controller",anonymous=True)
        rospy.loginfo("Satar car node……")
        
        self.light_color = 'green'
        self.limit_speed = -1
        self.line_pos = -1
        # 获取launch参数
        self.v0 = float(rospy.get_param('~v0',40.0))
        if self.v0 <= 0 and self.v0 >= 120:
            print('小车初始速度设置超出范围！')
            return -1

        self.bridge = CvBridge()
        # 订阅红绿灯节点发布的消息
        self.light_sub = rospy.Subscriber("/light_color", String, self.traffic_light_call_back)
        # 订阅图像节点发布的消息
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.cur_img_call_back)
        # 订阅识别线路节点的消息
        self.line_sub = rospy.Subscriber("/line_pos", Int8, self.line_pos_call_back)
        # 订阅限速节点发布的消息
        self.limit_sub = rospy.Subscriber("/max_speed", Int8, self.speed_limit_call_back)
        # 发布小车运动
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    
    # 订阅消息得到的红绿灯颜色
    def traffic_light_call_back(self, data):
        self.light_color = data.data
        print("traffic_light: {}".format(self.light_color))


    # 订阅消息得到的小车实时图像
    def cur_img_call_back(self, data):
        try:
            self.src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    # 订阅消息得到的限速牌的值：-1表示没有限速牌
    def speed_limit_call_back(self, data):
        self.limit_speed = data.data
        print("speed_limit: {}".format(self.limit_speed))

    
    # 订阅消息得到的中线位置
    def line_pos_call_back(self, data):
        self.line_pos = data.data
        print("line_pos: {}".format(self.line_pos))


    # 根据线的位置和图像中垂线计算角速度
    def calculate_turning_angle(self, image_center, line_center):
        if line_center/image_center > 0.94 and line_center/image_center < 1.06:
            
            return 0    # 保持直线行驶
        else:
            return self.set_angular(image_center, line_center)    # 设置角速度拐弯

    # 设置角度
    def set_angular(self,image_center,line_center):
        if self.v0 > 20 and self.v0 <= 30:
            angular = float((image_center - line_center) / image_center) / 1.5
        elif self.v0 > 30 and self.v0 <= 40:
            angular = float((image_center - line_center) / image_center) / 1.3
        elif self.v0 > 40 and self.v0 <= 50:
            angular = float((image_center - line_center) / image_center) / 1.2
        elif self.v0 > 50 and self.v0 <= 60:
            angular = float((image_center - line_center) / image_center) / 1.1
        elif self.v0 > 60 and self.v0 <= 70:
            angular = float((image_center - line_center) / image_center) / 1.0
        elif self.v0 > 70 and self.v0 <= 80:
            angular = float((image_center - line_center) / image_center) / 0.9
        elif self.v0 > 80 and self.v0 <= 90:
            angular = float((image_center - line_center) / image_center) / 0.80
        elif self.v0 > 90 and self.v0 <= 100:
            angular = float((image_center - line_center) / image_center) / 0.75
        elif self.v0 > 100 and self.v0 <= 110:
            angular = float((image_center - line_center) / image_center) / 0.72
        elif self.v0 > 110 and self.v0 <= 120:
            angular = float((image_center - line_center) / image_center) / 0.70
        else:
            angular = 0
        return angular

    # 获取红绿灯的颜色
    def get_light_color(self):
        return self.light_color

    # 获取限速速度
    def get_limit_velocity(self):
        return self.limit_speed
        
        
    # 发布运动命令
    def run(self, velocity, angle):
        move_cmd = Twist()
        move_cmd.linear.x = velocity
        move_cmd.linear.y = 0
        move_cmd.linear.z = 0
        move_cmd.angular.x = 0
        move_cmd.angular.y = 0
        move_cmd.angular.z = angle
        self.pub_cmd.publish(move_cmd)

    # # 获取小车即将要运动的线速度
    # def get_velocity(self):
    #     # 排除输入初速度负数情况
    #     if self.v0 < 0:
    #         return 0, False
    #     limit_speed = self.get_limit_velocity()
    #     if (limit_speed == -1):
    #         return self.v0, False   # 没有识别到限速牌，不进行限速，继续保持原速
    #     else:
    #         if self.v0 <= limit_speed:
    #             return self.v0, False    # 在限速内，不进行限速，继续保持原速
    #         else:
    #             # 先保持原速运动到识别不到限速牌的位置
    #             # while self.get_limit_velocity() != -1:
    #             #     return self.v0
    #             # # 然后在进行限速运动5s后恢复原速
    #             # time_start = time.time()
    #             # duration = 0
    #             # while duration < 5:
    #             #     self.run(velocity, angle)
    #             #     duration = time.time() - time_start
    #             return limit_speed, True    # 超出限速，需要限速，速度为限速值
            

    # 获取小车即将要运动的线速度
    def get_velocity(self):
        # 排除输入初速度负数情况
        if self.v0 < 0:
            return 0, False
        limit_speed = self.get_limit_velocity()
        if (limit_speed == -1):
            return self.v0, False   # -1没有识别到限速牌，不进行限速，继续保持原速
        else:
            if self.v0 <= limit_speed:
                return self.v0, False    # 在限速内，不进行限速，继续保持原速
            else:
                return limit_speed, True    # 超出限速，需要限速，速度为限速值


    # 小车的运动控制
    def start(self):

        color = self.get_light_color()

        if color == 'green':
            
            # 这是中线所在图像中的列坐标
            line_center = self.line_pos

            if line_center == -1:
                self.run(0, 0)
            else:
                image_center = self.src_image.shape[1] / 2
                angle = self.calculate_turning_angle(line_center, image_center)
                velocity, limit_flag = self.get_velocity()
                # self.run(velocity, angle)

                if limit_flag:  # 如果速度是限速后的
                    # 先保持原速运动到识别不到限速牌的位置
                    while self.get_limit_velocity() != -1:
                        self.run(self.v0, angle)
                    # 然后在进行限速运动5s后恢复原速
                    time_start = time.time()
                    duration = 0
                    while duration < 5:
                        self.run(velocity, angle)
                        duration = time.time() - time_start
                else:
                    # 不需要限速运动
                    self.run(velocity, angle)
        else:
            
            self.run(0, 0)


if __name__ == '__main__':

    try:
        
        car_mannager = RosCarController()
        car_mannager.start()
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting down car controller."
        cv2.destroyAllWindows()