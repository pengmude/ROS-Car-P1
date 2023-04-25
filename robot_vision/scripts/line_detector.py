#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 wechange tech.
# Developer: FuZhi Liu 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int8
import numpy as np
from dynamic_reconfigure.server import Server
from robot_vision.cfg import line_hsvConfig
from geometry_msgs.msg import Twist

class line_follow:
    
    def __init__(self): 
        #line center point X Axis coordinate
        self.center_point = 0
        self.light_color = 'green'
        self.max_speed = -1
        self.start_time = float(0)
        self.is_limit = False
        #define topic publisher and subscriber 
        self.bridge = CvBridge()
        self.light_sub = rospy.Subscriber("/light_color", String, self.callbacklight)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.limit_sub = rospy.Subscriber("/max_speed", Int8, self.callbackspeed)

        self.line_mask_pub = rospy.Publisher("/line_mask_image", Image, queue_size=1)
        self.line_result_pub = rospy.Publisher("/line_result_image", Image, queue_size=1)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.srv = Server(line_hsvConfig,self.dynamic_reconfigure_callback)
        
        # get param from launch file h_percentage
        self.test_mode = bool(rospy.get_param('~test_mode',False))
        self.h_lower = int(rospy.get_param('~h_lower',110))
        self.s_lower = int(rospy.get_param('~s_lower',50))
        self.v_lower = int(rospy.get_param('~v_lower',50))
        self.h_upper = int(rospy.get_param('~h_upper',130))
        self.s_upper = int(rospy.get_param('~s_upper',255))
        self.v_upper = int(rospy.get_param('~v_upper',255))
        self.v1 = float(rospy.get_param('~v1',0.56))
        self.v2 = float(rospy.get_param('~v2',0.46))
        self.v3 = float(rospy.get_param('~v3',0.38))
        self.v4 = float(rospy.get_param('~v4',0.30))

    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.h_lower = config.h_lower
        self.s_lower = config.s_lower
        self.v_lower = config.v_lower
        self.h_upper = config.h_upper
        self.s_upper = config.s_upper
        self.v_upper = config.v_upper
        self.v1 = config.v1
        self.v2 = config.v2
        self.v3 = config.v3
        self.v4 = config.v4
        return config

    def callbacklight(self,data):
        self.light_color = data.data
        print("light color:"+data.data)

    def callbackspeed(self,data):
        self.max_speed = data.data
        # print("max_speed:%d"%data.data)

    
    def callback(self,data):
        # convert ROS topic to CV image formart
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        # conver image color from RGB to HSV    
        self.hsv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_RGB2HSV)
        #set color mask min amd max value
        line_lower = np.array([self.h_lower,self.s_lower,self.v_lower])
        line_upper = np.array([self.h_upper,self.s_upper,self.v_upper])
        # get mask from color
        mask = cv2.inRange(self.hsv_image,line_lower,line_upper)
        # close operation to fit some little hole
        kernel = np.ones((9,9),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        # if test mode,output the center point HSV value
        res = self.cv_image.copy()
        if self.test_mode:
            cv2.circle(res, (self.hsv_image.shape[1]/2,self.hsv_image.shape[0]/2), 5, (0,0,255), 1)
            cv2.line(res,(self.hsv_image.shape[1]/2-10, self.hsv_image.shape[0]/2), (self.hsv_image.shape[1]/2+10,self.hsv_image.shape[0]/2), (0,0,255), 1)
            cv2.line(res,(self.hsv_image.shape[1]/2, self.hsv_image.shape[0]/2-10), (self.hsv_image.shape[1]/2, self.hsv_image.shape[0]/2+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%self.hsv_image[self.hsv_image.shape[0]/2,self.hsv_image.shape[1]/2])            
        else:
            for i in range(-40,20,10):
                point = np.nonzero(mask[mask.shape[0]*15/16 + i])             
                if len(point[0]) > 10:
                    self.center_point = int(np.mean(point))
                    cv2.circle(res, (self.center_point,self.hsv_image.shape[0]*15/16+i), 5, (0,0,255), 5)
                    cv2.line(res,(0, self.hsv_image.shape[0]*15/16+i), (self.hsv_image.shape[1], self.hsv_image.shape[0]*15/16+i), (0,255,0), 2)
                    break
        if self.center_point:
            self.twist_calculate(self.hsv_image.shape[1]/2,self.center_point)
        self.center_point = 0

        # show CV image in debug mode(need display device)
        # cv2.imshow("Image window", res)
        # cv2.imshow("Mask window", mask)
        # cv2.waitKey(3)

        # convert CV image to ROS topic and pub 
        try:
            img_msg = self.bridge.cv2_to_imgmsg(res, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.line_result_pub.publish(img_msg)
            img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.line_mask_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print e

    def twist_calculate(self,width,center):
        # t = time.time()
        center = float(center)
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

        if self.light_color != 'red':
            # print(time.time() - self.start_time)
            # 限速
            if 0 < int(self.max_speed) < 50:
                self.is_limit = True
                self.start_time = time.time()
                if center/width > 0.94 and center/width < 1.06:
                    self.twist.linear.x = self.v1 # 原本是0.2
                else:
                    self.twist.angular.z = ((width - center) / width) / 3.0
                    self.twist.linear.x = self.v2 # 原本是0.15
            else:
                if self.is_limit and time.time() - self.start_time < 6.0:
                    if center/width > 0.94 and center/width < 1.06:
                        self.twist.linear.x = self.v3 # 原本是0.2
                    else:
                        self.twist.angular.z = ((width - center) / width) / 3.0
                        self.twist.linear.x = self.v4 # 原本是0.15
                 
                else:
                    self.is_limit = False
                    if center/width > 0.94 and center/width < 1.06:
                        self.twist.linear.x = self.v1 # 原本是0.46
                    else:
                        self.twist.angular.z = ((width - center) / width) / 1.5
                        self.twist.linear.x = self.v2 # 原本是0.36
        self.pub_cmd.publish(self.twist)
                 
        # rospy.loginfo("运动控制:%.3f"%(time.time()-t))
    

if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("line_follow",anonymous=True)
        rospy.loginfo("Starting Line Follow node")
        line_follow()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()