#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
import numpy as np
import pytesseract
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from dynamic_reconfigure.server import Server
from robot_vision.cfg import limit_hsvConfig

class speed_limit:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.area_pub = rospy.Publisher("/limit_area", Image, queue_size=1)
        self.result_pub = rospy.Publisher("/limit_result", Image, queue_size=1)
        self.limit_pub = rospy.Publisher("/max_speed", Int8, queue_size=1)
        self.srv = Server(limit_hsvConfig,self.dynamic_reconfigure_callback)
        # get param from launch file h_percentage
        self.limit_area_r =  int(rospy.get_param('~limit_area_r',-5))
        self.threshold_min = int(rospy.get_param('~threshold_min',50))
        self.threshold_max = int(rospy.get_param('~threshold_max',255))

    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.limit_area_r = config.limit_area_r
        self.threshold_min = config.threshold_min
        self.threshold_max = config.threshold_max
        return config

    def callback(self,data):
        # t = time.time()
        # 第一步：图像处理，提取数字区域
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_rgb = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print e

        # imPath = "/home/bingda/3.png"
        # gray_img = cv2.imread(imPath, cv2.IMREAD_COLOR)
        gray_img = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        # ret, bin_img = cv2.threshold(gray_img,0,255,cv2.THRESH_BINARY_INV | cv2.THRESH_TRIANGLE)
        circles = cv2.HoughCircles(gray_img,cv2.HOUGH_GRADIENT,1,minDist=600,param1=240,param2=80,minRadius=10,maxRadius=60)
        # print("duration mid:%.3f"%(time.time()-t))
        img = self.cv_image.copy()
        # is_limit = False
        self.mask = np.ones_like(self.cv_image) * 255
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # rows, cols, channel = self.cv_image.shape
                cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),4)
                cv2.circle(img,(i[0],i[1]),2,(0,0,255),4)
                
                roi = np.zeros(self.cv_image.shape[:2], np.uint8)
                roi = cv2.circle(roi,(int(i[0]),int(i[1])), int(i[2]+self.limit_area_r), 255, cv2.FILLED)
                
                self.mask = cv2.bitwise_and(self.mask, self.cv_image, mask=roi) + cv2.bitwise_and(self.mask, self.mask, mask=~roi)  
                self.mask = cv2.cvtColor(self.mask,cv2.COLOR_BGR2GRAY)
                _, self.mask = cv2.threshold(self.mask,self.threshold_min,self.threshold_max,cv2.THRESH_BINARY)
                self.mask = self.mask[(i[1]-i[2]):(i[1]+i[2]),(i[0]-i[2]):(i[0]+i[2])]
                self.mask = cv2.blur(self.mask,(3,3))
                self.mask = cv2.resize(self.mask,None,fx=2,fy=2,interpolation=cv2.INTER_LINEAR)
                print("")
                # cv2.imwrite('/home/bingda/catkin_ws/src/robot_vision/scripts/template_40.png', self.mask, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                
                #print("duration mid:%.3f"%(time.time()-t))
                # self.mask = self.mask[:self.mask.shape[1],self.mask.shape[0]/2:]
                # self.mask = self.mask[self.mask.shape[1]*1/4:self.mask.shape[1]*3/4,self.mask.shape[0]/2:]
                # speed_limit = pytesseract.image_to_string(self.mask,lang='eng',config='--psm 7 --oem 3 -c tessedit_char_whitelist=0123456789')
                    
                # if speed_limit is not None:
                #     is_limit = True
                #     print("max speed:"+speed_limit)
                # else:
                #     print("max speed:999")

        else:
            print("")

        try: 
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.area_pub.publish(img_msg)

            if len(self.mask.shape) == 3:
                self.mask = self.mask[:112,:112,0]
            # print("shape:",self.mask.shape)
            
        except CvBridgeError as e:
            print e
        
        # 第二步：识别数字
        self.match_limit_shape(self.mask)
        # print("duration end:%.3f"%(time.time()-t))

    def match_limit_shape(self,speed_image):

        speed_image = cv2.cvtColor(speed_image,cv2.COLOR_BGR2RGB)
        _, speed_image = cv2.threshold(speed_image,0,255,cv2.THRESH_BINARY_INV)
        speed_image = cv2.blur(speed_image,(3,3)) # 圆滑
        speed_image = speed_image[:,:,0]

        img_msg = self.bridge.cv2_to_imgmsg(speed_image, encoding="passthrough")
        img_msg.header.stamp = rospy.Time.now()
        self.result_pub.publish(img_msg)

        template_img = cv2.imread('/home/bingda/catkin_ws/src/robot_vision/scripts/template_40.png')
        template_img = cv2.cvtColor(template_img,cv2.COLOR_BGR2RGB)
        _, template_img = cv2.threshold(template_img,0,255,cv2.THRESH_BINARY_INV)
        template_img = template_img[:,:,0]
        # print("img2:",speed_image.shape)

        contours1, _ = cv2.findContours(speed_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(template_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        if contours1:
            res_right = cv2.matchShapes(contours1[0], contours2[0], cv2.CONTOURS_MATCH_I3, 0.0)
            res_left = cv2.matchShapes(contours1[1], contours2[1], cv2.CONTOURS_MATCH_I3, 0.0)
            matching_value = max(res_right,res_left)
            # print("matching_value:%s"%matching_value)
            if matching_value < 2:
                self.max_speed = 40
        else:
            self.max_speed = -1
        print("max_speed:%s"%self.max_speed)
        self.limit_pub.publish(self.max_speed)


if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("speed_limit")
        rospy.loginfo("Starting Speed limit node")
        #rate = rospy.Rate(30)
        speed_limit()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()