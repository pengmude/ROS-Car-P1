#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from robot_vision.cfg import limit1_hsvConfig
from dynamic_reconfigure.server import Server
 
class SpeedLimitDetector():
 
    def __init__(self):
	
        # 加载模板图片
        self.templates = []
		# 30~120
        for i in range(30, 130, 10): 
            filename = os.path.join(os.path.dirname(__file__), "template_imgs/template_{}.png".format(i))
            # print(filename)
            img = cv2.imread(filename, 0)
            if img is not None:
                print(img.shape)
            else:
                print("Can not read this image !")

            img = cv2.resize(img, (96, 96))
            img = cv2.bitwise_not(img)
            self.templates.append(img)
        


        # 创建ROS节点和订阅者
        rospy.init_node('speed_limit_detector')
        self.img_sub = rospy.Subscriber('/image_raw', Image, self.image_callback) 
        self.circle_img_pub = rospy.Publisher("/limit_circle_area", Image, queue_size=1)
        self.cv_bridge = CvBridge()
        
        # 动态调参回调函数
        self.srv = Server(limit1_hsvConfig,self.dynamic_reconfigure_callback)
    
    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.h_lower = config.h_lower
        self.s_lower = config.s_lower
        self.v_lower = config.v_lower
        self.h_upper = config.h_upper
        self.s_upper = config.s_upper
        self.v_upper = config.v_upper
        return config
    
    def image_callback(self, img_msg):
        # 将ROS图像消息转换为OpenCV图像格式
        cv_img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # 在图像中查找圆
        gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 100, param1=240, param2=80, minRadius=10, maxRadius=int(min(gray_img.shape)/3))
        # circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=int(min(gray_img.shape)/2))
 
        if circles is not None:
            # 找到圆后，将包含数字的圆形区域截取出来
            img, x, y, r = self.circle_crop(cv_img, circles[0])
            # 识别出数字
            digit, value= self.detect_digit(img)
            # 显示结果
            if digit is not None:
                rospy.loginfo("识别到限速: {} km/h".format(digit))
                cv2.putText(cv_img, "Speed Limit: {}".format(digit), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(cv_img, "Matching value: {}".format(value), (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv2.putText(cv_img, f"Matching value: {value:.2f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                rospy.loginfo("识别到限速: 未知速度")
                cv2.putText(cv_img, "Speed Limit: ", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(cv_img, "Matching value: ", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print("\n")
            
        circle_img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        circle_img_msg.header.stamp = rospy.Time.now()
        self.circle_img_pub.publish(circle_img_msg)
 
    def circle_crop(self, img, circle):

        # 裁剪出圆形限速牌的最小外接正方形
        w = int(circle[0][0])
        h = int(circle[0][1])
        r = int(circle[0][2] * 1.0)
        cropped_img = img[h-r:h+r, w-r:w+r]
        
        # 将识别到的圆形限速牌轮廓标注在图像上
        cv2.circle(img, (w, h), r+5, (0,255,0), 3)

        # 消除正方形与内接圆不重合的四个角区域，这样不会影响识别，像素设为255即可
        h1, w1, _ = cropped_img.shape
        circle_area_img = cropped_img.copy()
        for y1 in range (h1):
                for x1 in range (w1):
                    if (x1-r)**2 + (y1-r)**2 > r**2:
                        circle_area_img[y1][x1] = 255
        return circle_area_img, w, h, r
 
    def detect_digit(self, img):

        # 缩小图像到28*28像素
        img = cv2.resize(img, (96, 96))

        # 过滤得到要消除的红色边框
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([self.h_lower,self.s_lower,self.v_lower])
        upper_red = np.array([self.h_upper,self.s_upper,self.v_upper])
        mask = cv2.inRange(hsv_img, lower_red, upper_red)
        
        # 显示图像
        cv2.imshow('红色边框', mask)
        cv2.waitKey(1)


        # 取反后当做掩膜作与运算，红色区域变为黑色，再将它与仅有红色区域的图像（mask）相加，得到消除红色的图
        mask_inv = cv2.bitwise_not(mask)
        img = cv2.bitwise_and(img, img, mask=mask_inv)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        img = cv2.add(img, mask)
                
        # 显示图像
        # cv2.imshow('消除红色边框', img)
        # cv2.waitKey(1)

        # 二值化图像
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        _, threshold_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # 高斯滤波消除噪点
        # threshold_img = cv2.GaussianBlur(threshold_img, (5,5), 0)

        kernel = np.ones((3,3), np.uint8)
        open_img = cv2.morphologyEx(threshold_img, cv2.MORPH_OPEN, kernel)
        threshold_img = open_img
        
        # 显示图像
        cv2.imshow('最终图像', threshold_img)
        cv2.waitKey(1)

        # 使用模板匹配获取数字
        max_match = None
        match_idx = None
        for i, template in enumerate(self.templates):

            result = cv2.matchTemplate(threshold_img, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            if max_match is None or max_val > max_match:
                max_match = max_val
                match_idx = i * 10
        # 匹配相似值保留2位小数
        max_match = round(max_match, 2)
        print("匹配值:{}".format(max_match))

        if max_match > 0.3:
            if match_idx is not None:
                cv2.imshow('模板图像', self.templates[match_idx/10])
                cv2.waitKey(1)
            return match_idx + 30, max_match
        else:
            return None, max_match

 
    def run(self):
        rospy.spin()
 
    def __del__(self):
        self.sess.close()
 
if __name__ == '__main__':
    detector = SpeedLimitDetector()
    detector.run()