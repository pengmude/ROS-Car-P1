#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from dynamic_reconfigure.server import Server
from robot_vision.cfg import light_hsvConfig

class light_color:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.mask_pub = rospy.Publisher("/light_mask", Image, queue_size=1)
        self.result_pub = rospy.Publisher("/light_result", Image, queue_size=1)
        self.color_pub = rospy.Publisher("/light_color", String, queue_size=1)
        self.srv = Server(light_hsvConfig,self.dynamic_reconfigure_callback)
        # get param from launch file h_percentage
        self.test_mode = bool(rospy.get_param('~test_mode',False))
        self.h_lower = int(rospy.get_param('~h_lower',110))
        self.s_lower = int(rospy.get_param('~s_lower',50))
        self.v_lower = int(rospy.get_param('~v_lower',50))

        self.h_upper = int(rospy.get_param('~h_upper',130))
        self.s_upper = int(rospy.get_param('~s_upper',255))
        self.v_upper = int(rospy.get_param('~v_upper',255))

        self.col_1 = int(rospy.get_param('~col_1',260))
        self.col_2 = int(rospy.get_param('~col_2',380))
        self.row_1 = int(rospy.get_param('~row_1',60))
        self.row_2 = int(rospy.get_param('~row_2',180))

    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.h_lower = config.h_lower
        self.s_lower = config.s_lower
        self.v_lower = config.v_lower
        self.h_upper = config.h_upper
        self.s_upper = config.s_upper
        self.v_upper = config.v_upper
 
        self.col_1 = config.col_1
        self.col_2 = config.col_2
        self.row_1 = config.row_1
        self.row_2 = config.row_2

        if config.col_1 > config.col_2: 
            temp = config.col_1 
            config.col_2 = config.col_1
            config.col_1 = temp
        if config.row_1 > config.row_2: 
            temp = config.row_1 
            config.row_2 = config.row_1
            config.row_1 = temp
        
        return config

    def callback(self,data):
        #t = time.time()
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
        kernel = np.ones((25,25),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        result_image = self.cv_image.copy()
        cv2.rectangle(result_image,(self.col_1,self.row_1),(self.col_2,self.row_2),(0,255,255),2)
        self.flag = False
        #print("height:",mask.shape[0], " width:",mask.shape[1]) #width=320,height=240
        for i in range(self.row_1,self.row_2,10):
                #point = np.nonzero(mask[mask.shape[0]*self.numerator/self.denominator + i])
                point = np.nonzero(mask[i]) 
                #print("point size:",len(point[0]))            
                if len(point[0]) > 10:
                    self.center_point = int(np.mean(point))
                    if self.col_1 < self.center_point and self.center_point < self.col_2:
                        self.flag = True
                        self.color_pub.publish('red')
                        print("Red")
                        cv2.circle(result_image, (self.center_point,i), 5, (0,0,255), 5)
                        break

        try:
            img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub.publish(img_msg) 

            img_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.result_pub.publish(img_msg) 

            if not self.flag:
                print("Not red")
                self.color_pub.publish('green')
            
        except CvBridgeError as e:
            print e
        
        #rospy.loginfo("duration:%.3f"%(time.time()-t))

if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("traffic_light")
        rospy.loginfo("Starting Light color node")
        light_color()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()