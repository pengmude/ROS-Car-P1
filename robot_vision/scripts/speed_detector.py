#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os
from robot_vision.cfg import limit_hsvConfig
from dynamic_reconfigure.server import Server
 
class SpeedLimitDetector():
 
    def __init__(self):
        # 加载模板图片
        self.templates = []
		# 30~120
        for i in range(30, 130, 10): 
            filename = os.path.join(os.path.dirname(__file__), "../data/template_imgs/template_{}.png".format(i))
            img = cv2.imread(filename, 0)
            if img is not None:
                print(img.shape)
            else:
                print("Can not read this image !")
            # img = cv2.resize(img, (72, 72))
            # img = cv2.bitwise_not(img)
            self.templates.append(img)

        # 创建ROS节点和订阅者
        rospy.init_node('speed_limit')

        # 获取launch参数
        self.test_mode = bool(rospy.get_param('~test_mode', False)) 
        print('test_mode: {}'.format(self.test_mode))
        
        self.img_sub = rospy.Subscriber('/image_raw', Image, self.image_callback) 
        self.circle_img_pub = rospy.Publisher("/limit_circle_area", Image, queue_size=1)
        self.speed_limit_pub = rospy.Publisher("/max_speed", Int8, queue_size=1)
        self.cv_bridge = CvBridge()

        # 动态调参回调函数
        self.srv = Server(limit_hsvConfig,self.dynamic_reconfigure_callback)
    
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
        
        if circles is not None:
            
            for _, circle in enumerate(circles):

                if self.test_mode:
                    self.create_template_img(cv_img, circle)
                    break
                else:
                    # 找到圆后，将包含数字的圆形区域截取出来
                    img, x, y, r = self.circle_crop(cv_img, circle)
                    # 识别出数字
                    digit_index, value = self.detect_digit(img)
                    print('index: {},   value: {}'.format(digit_index, value))
                    # 将识别到的圆形限速牌轮廓标注在图像上
                    cv2.circle(cv_img, (x, y), r+20, (0,255,0), 3)
                    cv2.putText(cv_img, "{}".format(value), (x-80, y+150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # 显示匹配的模板图像
                    # cv2.imshow('模板图像', self.templates[digit_index])
                    # cv2.waitKey(1)

                    # 匹配相似值保留2位小数
                    value = round(value, 2)
                    # 显示限速结果和匹配值，匹配值介于0~1之间，越小匹配度越高和匹配值，匹配值介于0~1之间，越小匹配度越高
                    if value < 0.45:
                        rospy.loginfo("识别到限速: {} km/h".format(digit_index*10+30))
                        rospy.loginfo("匹配值： {}".format(value))
                        cv2.putText(cv_img, "Speed Limit: {}".format(digit_index*10+30), (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(cv_img, "Matching value: {}".format(value), (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        self.speed_limit_pub.publish(digit_index*10+30)
                    else:
                        rospy.loginfo("识别到限速: 未知速度")
                        cv2.putText(cv_img, "Speed Limit: ", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(cv_img, "Matching value: ", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        self.speed_limit_pub.publish(-1)
                    # 发布识别结果图像
                    circle_img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    circle_img_msg.header.stamp = rospy.Time.now()
                    self.circle_img_pub.publish(circle_img_msg)
                    print("\n")
        else:
            self.speed_limit_pub.publish(-1)
            print('没有识别到限速牌的圆形特征')
        
        


    def create_template_img(self, img, circle):
        # 裁剪出圆形限速牌的最小外接正方形
        w = int(circle[0][0])
        h = int(circle[0][1])
        r = int(circle[0][2] * 1.0)
        cropped_img = img[h-r:h+r, w-r:w+r]
        # # 消除正方形与内接圆不重合的四个角区域，这样不会影响识别，像素设为255即可
        # h1, w1, _ = cropped_img.shape
        # for y1 in range (h1):
        #         for x1 in range (w1):
        #             if (x1-r)**2 + (y1-r)**2 > r**2:
        #                 cropped_img[y1][x1] = 0
        
        
        # 过滤得到要消除的红色边框
        # hsv_img = cv2.cvtColor(circle_area_img, cv2.COLOR_BGR2HSV)
        # lower_red = np.array([self.h_lower,self.s_lower,self.v_lower])
        # upper_red = np.array([self.h_upper,self.s_upper,self.v_upper])
        # mask = cv2.inRange(hsv_img, lower_red, upper_red)
        
        # 显示图像
        # cv2.imshow('红色边框', mask)
        # cv2.waitKey(1)

        # 取反后当做掩膜作与运算，红色区域变为黑色，再将它与仅有红色区域的图像（mask）相加，得到消除红色的图
        # mask_inv = cv2.bitwise_not(mask)
        # img = cv2.bitwise_and(img, img, mask=mask_inv)
        # mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # img = cv2.add(img, mask)

        # 显示图像
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        _, threshold_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        cv2.imshow('template:', threshold_img)
        cv2.waitKey(1)

        circle_img_msg = self.cv_bridge.cv2_to_imgmsg(threshold_img, encoding="mono8")
        circle_img_msg.header.stamp = rospy.Time.now()
        self.circle_img_pub.publish(circle_img_msg)


    def circle_crop(self, img, circle):
        # 裁剪出圆形限速牌的最小外接正方形
        w = int(circle[0][0])
        h = int(circle[0][1])
        r = int(circle[0][2] * 1.0)
        
        cropped_img = []
        tmp_img = img.copy()
        h1, w1, _ = tmp_img.shape

        if (h-r-40 >= 0) and (h+r+40 <= h1) and (w-r-40 >= 0) and (w+r+40 <= w1):
                cropped_img = tmp_img[h-r-40:h+r+40, w-r-40:w+r+40]
                # # # 消除正方形与内接圆不重合的四个角区域，这样不会影响识别，像素设为255即可
                # h1, w1, _ = cropped_img.shape
                # for y1 in range (h1):
                #         for x1 in range (w1):
                #             if (x1-r)**2 + (y1-r)**2 > r**2:
                #                 cropped_img[y1][x1] = 0
        else:
            cropped_img = tmp_img[h-r:h+r, w-r:w+r]
        
        # 消除正方形与内接圆不重合的四个角区域，这样不会影响识别，像素设为255即可
        # h1, w1, _ = tmp_img.shape
        # for y1 in range (h1):
        #         for x1 in range (w1):
        #             if (x1-r)**2 + (y1-r)**2 > (r)**2:
        #                 cropped_img[y1][x1] = 0
    
        return cropped_img, w, h, r
 

    def detect_digit(self, cropped_img):

        # 灰度化图像
        gray_img = cv2.cvtColor(cropped_img,cv2.COLOR_BGR2GRAY)

        # 二值化图像
        _, threshold_img = cv2.threshold(gray_img, 150, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # 显示图像
        # cv2.imshow('最终图像', threshold_img)
        # cv2.waitKey(1)

        # 使用模板匹配获取数字
        res = [0]*len(self.templates)
        for i, template in enumerate(self.templates):
            result = cv2.matchTemplate(threshold_img, template, cv2.TM_SQDIFF_NORMED)
            # cv2.TM_SQDIFF_NORMED匹配方法得到的结果越小，匹配度越高,范围[0,1]
            min_val, max_val, minLoc, max_loc = cv2.minMaxLoc(result)
            res[i] = min_val

        min_value = 1
        min_index = 0
        for i, value in enumerate(res):
            if value < min_value:
                min_value = value
                min_index = i
                # print("index000: {},  value000: {}".format(i, value))
        return min_index, min_value

 
    def run(self):
        rospy.spin()
 
    def __del__(self):
        self.sess.close()
 
if __name__ == '__main__':
    detector = SpeedLimitDetector()
    detector.run()