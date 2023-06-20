#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
from dynamic_reconfigure.server import Server
from robot_vision.cfg import line_hsvConfig

class line_follow:
    
    def __init__(self): 

        self.center_point = -1

        # get param from launch file h_percentage
        self.test_mode = bool(rospy.get_param('~test_mode',False))
        self.h_lower = int(rospy.get_param('~h_lower',110))
        self.s_lower = int(rospy.get_param('~s_lower',50))
        self.v_lower = int(rospy.get_param('~v_lower',50))
        self.h_upper = int(rospy.get_param('~h_upper',130))
        self.s_upper = int(rospy.get_param('~s_upper',255))
        self.v_upper = int(rospy.get_param('~v_upper',255))
        self.green_line = int(rospy.get_param('~green_line',15)) 
        self.offset_pos = int(rospy.get_param('~offset_pos',0))

        # 定义订阅器和发布器
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.line_mask_pub = rospy.Publisher("/line_mask_image", Image, queue_size=1)
        self.line_result_pub = rospy.Publisher("/line_result_image", Image, queue_size=1)
        self.line_pos_pub = rospy.Publisher("/line_pos", Int32, queue_size=1)
        self.srv = Server(line_hsvConfig, self.dynamic_reconfigure_callback)
        

    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.h_lower = config.h_lower
        self.s_lower = config.s_lower
        self.v_lower = config.v_lower
        self.h_upper = config.h_upper
        self.s_upper = config.s_upper
        self.v_upper = config.v_upper
        return config

    
    def callback(self,data):
        # 将ROS消息数据转化为OpenCV能处理的格式
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        # 将RGB图像转成HSV图像
        self.hsv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_RGB2HSV)
        # 设置HSV范围
        line_lower = np.array([self.h_lower,self.s_lower,self.v_lower])
        line_upper = np.array([self.h_upper,self.s_upper,self.v_upper])
        # 通过特定范围的HSV值过滤得到线的部分
        mask = cv2.inRange(self.hsv_image,line_lower,line_upper)
        # 闭操作修复中线的小孔
        kernel = np.ones((9,9),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        # 如果是test模式，输出准星的HSV值，用于配置lanuch文件的HSV参数
        res = self.cv_image
        if self.test_mode:
            # 测试模式用来对焦巡线获得HSV颜色分量 
            cv2.circle(res, (self.hsv_image.shape[1]/2,self.hsv_image.shape[0]/2), 5, (0,0,255), 1)
            cv2.line(res,(self.hsv_image.shape[1]/2-10, self.hsv_image.shape[0]/2), (self.hsv_image.shape[1]/2+10,self.hsv_image.shape[0]/2), (0,0,255), 1)
            cv2.line(res,(self.hsv_image.shape[1]/2, self.hsv_image.shape[0]/2-10), (self.hsv_image.shape[1]/2, self.hsv_image.shape[0]/2+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%self.hsv_image[self.hsv_image.shape[0]/2,self.hsv_image.shape[1]/2])            
        else:
            if self.green_line == 15:
                for i in range(-40,20,10):
                    point = np.nonzero(mask[mask.shape[0]*15/16 + i])             
                    if len(point[0]) > 10:
                        self.center_point = int(np.mean(point))
                        self.center_point = self.center_point + self.offset_pos
                        cv2.circle(res, (self.center_point,self.hsv_image.shape[0]*15/16+i), 5, (0,0,255), 5)
                        cv2.line(res,(0, self.hsv_image.shape[0]*15/16+i), (self.hsv_image.shape[1], self.hsv_image.shape[0]*15/16+i), (0,255,0), 2)
                        break
            else:
                for i in range(-40,20,10):
                    point = np.nonzero(mask[mask.shape[0]*self.green_line/16 + i])             
                    if len(point[0]) > 10:
                        self.center_point = int(np.mean(point))
                        self.center_point = self.center_point + self.offset_pos
                        cv2.circle(res, (self.center_point,self.hsv_image.shape[0]*self.green_line/16+i), 5, (0,0,255), 5)
                        cv2.line(res,(0, self.hsv_image.shape[0]*self.green_line/16+i), (self.hsv_image.shape[1], self.hsv_image.shape[0]*self.green_line/16+i), (0,255,0), 2)
                        break
        # 发布识别到的中点
        self.line_pos_pub.publish(self.center_point)
        # print(self.center_point)
        self.center_point = -1

        # 将OpenCV格式图像转化为ROS消息格式，然后发布出去
        try:
            img_msg = self.bridge.cv2_to_imgmsg(res, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.line_result_pub.publish(img_msg)
            img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.line_mask_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("line_detector")
        rospy.loginfo("Starting Line Follow node")
        line_follow()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down line_detector node."
        cv2.destroyAllWindows()