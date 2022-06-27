#!/usr/bin/python3
# -*- coding: utf-8 -*-

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

from os import system
import realsense2_camera as realcamera
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import pyrealsense2 as rs
import numpy as np
import rospy
import time
import os
import cv2
import pyzbar.pyzbar as pyzbar
from matplotlib import pyplot as plt
from io import BytesIO
import math

speed = 0 #msg.linear.x
angle = 0 #msg.angular.z

limit = 2000 # lidar의 물체인식 범위 값 mm 단위

# 물체인식을 " 123456789" 문자열로 표현하기 위한 변수 초기값
data_all = [] 
view_data_all = [] 
data_data_all = []
data_data_0 = []
data_data_0_np = []
data_data_1 = []
data_data_1_np = []
data_data_01 = []
data_data_all = []

## mapping = ["매핑준비중", "매핑진행중", "매핑종료", "매핑완료"] << 로봇이 mapping 을 할 때, terminal에 mapping 값이 변하는 것을 보기 위해 임의로 지정함
mapping = "매핑준비중" 

qr_moving = "QR대기중" 
qr_target = "QR_X"
qr_moving_step = "QR대기중"
run_direction = "정면" # 로봇의 방향

my_distance = 0
my_angle = 0


# 로봇이 QR를 찍을 때, 이동하는 거리(meter)와 각도(angle) 지정
QR_A_distance = 2
QR_A_angle = 40 
QR_B_distance = 2.5
QR_B_angle = 70
QR_C_distance = 2
QR_C_angle = 115

# QR을 찍을 cam지정과 사이즈 지정
cap_1 = cv2.VideoCapture(10)

cap_1.set(cv2.CAP_PROP_FRAME_HEIGHT,200)
cap_1.set(cv2.CAP_PROP_FRAME_WIDTH,320)


def lidar_pose(data): # rospy.Subscriber("/odom",Odometry,lidar_pose) L515 현재위치와 각도 값을 쓰기 위한 함수 
    global position_x, position_y, orientation_x, orientation_y, orientation_z, orientation_w, roll_x, pitch_y, yaw_z
    try : 
        
        # /odom data 값 추출 Quaternion(쿼터니언)방식으로 표현
        orientation_x = round(data.pose.pose.orientation.x,1)
        orientation_y = round(data.pose.pose.orientation.y,1)
        orientation_z = round(data.pose.pose.orientation.z,1)
        orientation_w = round(data.pose.pose.orientation.w,1) 
        
        # /odom data 안의 position(위치) 값
        position_x = round((data.pose.pose.position.x),1)
        position_y = round((data.pose.pose.position.y),1) 
        
        # roll = x 축 중심으로 회전(degree 값 사용)
        t0 = +2.0 * (orientation_w * orientation_x + orientation_y * orientation_z) 
        t1 = +1.0 - 2.0 * (orientation_x * orientation_x + orientation_y * orientation_y)
        roll_x = math.atan2(t0, t1)
        roll_x = round((roll_x * 180 / math.pi),3)
        
        # pitch = y축 중심으로 회전
        t2 = +2.0 * (orientation_w * orientation_y - orientation_z * orientation_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        pitch_y = round((pitch_y * 180 / math.pi),3)

        # yaw = z축 중심으로 회전
        t3 = +2.0 * (orientation_w * orientation_z + orientation_x * orientation_y)
        t4 = +1.0 - 2.0 * (orientation_y * orientation_y + orientation_z * orientation_z)
        yaw_z = math.atan2(t3, t4)
        yaw_z = round((yaw_z * 180 / math.pi),3)

        return roll_x, pitch_y, yaw_z
    except : pass

def cam_lidar_read(data): # rospy.Subscriber("/camera/depth/image_rect_raw",Image,cam_lidar_read) L515의 depth 값을 받아오기 위한 함수 
    global data_data_0, data_data_1, angle, speed, mapping, qr_moving, qr_target, qr_moving_step, my_distance, my_angle, run_direction
    global barcode_data_product_QR, barcode_type_product_QR, decoded_product_QR, retval_1, frame_1, cap_1


    # CAM을 이용하여 QR코드 인식 라이브러리 사용
    retval_1, frame_1 = cap_1.read()

    decoded_product_QR = pyzbar.decode(frame_1)
    
    for _ in decoded_product_QR:
        a, b, c, d = _.rect 
        barcode_data_product_QR = _.data.decode("utf-8")
        barcode_type_product_QR = _.type
        cv2.rectangle(frame_1, (a, b), (a + c, b + d), (0, 0, 255), 2)
        text_1 = '%s (%s)' % (barcode_data_product_QR, barcode_type_product_QR)
        cv2.putText(frame_1, text_1, (a, b), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        
    if decoded_product_QR == [] :
        barcode_data_product_QR = "QR_X"
    
    cv2.imshow('Video', frame_1) 
 
    cv2.waitKey(1) 



    
    try :
        
        view_data_all.clear()
        data_data_0.clear()
        data_data_1.clear()

        # L515 기기의 /camera/depth/image_rect_raw data 값 추출 및 변환, 총 307200개의 값 출력됨 >> numpy 함수를 사용하여 값 추출 시  값 추출 속도 최적화 시킴
        data_data = data.data 
        data_data_arr = bytearray(data_data) 

        data_data_0 = data_data_arr[0::2] 
        data_data_0_np = np.array(data_data_0)

        data_data_1 = data_data_arr[1::2] 
        data_data_1_np = np.array(data_data_1)
        data_data_1_np = data_data_1_np * 256

        data_data_01 = np.array([data_data_0_np,data_data_1_np]) 
        
        data_data_all = data_data_01.sum(axis=0) 
        
        xy = 0
        
        add_num = 0 
        add_edge_num = 0 
        add_edge_remain_num = 0 
        
        os.system('clear') 

        print("--------------------------------")

        # y는 열의 개념으로 사용 x는 행의 개념으로 사용(12열 32행으로 표현)하여 물체가 인식될 시 물체의 depth에 따라 " 123456789"로 표현함
        for y in range(480) :

            xy = y * 640

            if y % 40 == 0 :
                view_data = ""
                for x in range(640) :
                    xy = xy + 1
                    if x % 20 == 0 : 
                        if data_data_all[xy]  > limit : view_data += " " 
                        if data_data_all[xy] <= limit : view_data += " 1234567899"[int(data_data_all[xy])//int(limit/10)] 
                view_data_all.append(view_data)
                
                if y >= 0 and y < 360 :
                    for a in range(1,32):
                        if view_data[a] == " " : add_num = add_num 
                        else : add_num = add_num + int(view_data[a])
                    for a in range(30,32):
                        if view_data[a] == " " : add_edge_num = add_edge_num
                        else : add_edge_num = add_edge_num + int(view_data[a])
                    for a in range(1,20):
                        if view_data[a] == " " : add_edge_remain_num = add_edge_remain_num
                        else : add_edge_remain_num = add_edge_remain_num + int(view_data[a])

        for y in range(12):
            print(view_data_all[y])
            
        #mapping 완료 후 로봇이 출발점 position 값과 일치하도록 변수 값 지정(test 결과 각도는 11도 거리는 0.64 만큼의 오차 발생)
        my_angle = int(yaw_z)
        my_distance = math.sqrt((position_x * position_x) + (position_y * position_y))
        
        error_angle = 23
        error_distance = 0.50

        my_add_error_angle = my_angle + error_angle
        my_add_error_distange = my_distance - error_distance

        #### 로봇이 움직이고 mapping 시작####
        
        ## (매핑준비중 = 1m 이상 움직이기 전), PID 제어를 하기 위해 장애물이 인식 및 회피 시 속도와 각도를 제어함(부드럽고 안정적인 주행을 위함)
        if mapping == "매핑준비중" or mapping == "매핑진행중": 
            if add_edge_num == 0 :
                if add_edge_remain_num == 0 :
                    speed = speed - 0.007
                    angle = angle - 0.007
                if add_edge_remain_num >  0 :
                    speed = speed - 0.007
                    angle = angle + 0.007
            if add_edge_num  > 0 :
                if add_edge_remain_num == 0 :
                    speed = speed + 0.02
                    if angle >  0.01 : angle = angle - 0.007
                    if angle < -0.01 : angle = angle + 0.007
                if add_edge_remain_num >  0 :
                    speed = speed - 0.0015
                    angle = angle + 0.007

        if mapping == "매핑준비중" and my_distance > 1 : mapping = "매핑진행중"
            
        ## (매핑진행중 =  1m 이상 움직인 후 매핑 진행중)
        if mapping == "매핑진행중" and my_add_error_distange < 0.3 :
            speed = 0
            angle = 0
            mapping = "매핑종료"

        ## ( 매핑종료 = 매핑이 끝난 후 처음위치 10cm 이내 도착시 정지 및 각도조절 )
        if mapping == "매핑종료" :
            if not(my_add_error_angle == 0) :
                if my_add_error_angle > 0  : angle = -0.3
                if my_add_error_angle < 0  : angle =  0.3
            if    (my_add_error_angle == 0) : 
                angle = 0
                mapping = "매핑완료"

        ## (매핑완료 = 매핑종료 후 사물QR 대기)
        if mapping == "매핑완료" :
            if barcode_data_product_QR != "QR_X" : 
                qr_moving = "QR진행중"
                qr_moving_step = "각도변경중"
                if qr_target == "QR_X":
                    if barcode_data_product_QR == "A": qr_target = "QR_A"
                    if barcode_data_product_QR == "B": qr_target = "QR_B"
                    if barcode_data_product_QR == "C": qr_target = "QR_C"
                    
            if qr_moving_step == "출발지복귀중" and barcode_data_product_QR == "QR_X" : qr_target = "QR_X"

            if qr_moving == "QR진행중" :
                ## QR 코드가 "A"일 때, 지정해둔 A의 구역을 가기위해 지정 각도와 거리 이동
                if qr_target == "QR_A" :
                    if qr_moving_step == "각도변경중" : 
                        if not(abs(my_add_error_angle - QR_A_angle) < 10) : 
                            if (my_add_error_angle - QR_A_angle) > 0 : angle = -0.15
                            if (my_add_error_angle - QR_A_angle) < 0 : angle =  0.15
                        if    (abs(my_add_error_angle - QR_A_angle) < 10) :
                            angle = 0
                            qr_moving_step = "목적지이동중"

                    if qr_moving_step == "목적지이동중":
                        if not(abs(QR_A_distance - my_add_error_distange) < 0.1) : speed = 0.3
                        if    (abs(QR_A_distance - my_add_error_distange) < 0.1) :
                            speed = 0
                            qr_moving_step = "출발지복귀중"
                            
                ## QR 코드가 "B"일 때, 지정해둔 B의 구역을 가기위해 지정 각도와 거리 이동
                if qr_target == "QR_B" :
                    if qr_moving_step == "각도변경중" : 
                        if not(abs(my_add_error_angle - QR_B_angle) < 10) : 
                            if (my_add_error_angle - QR_B_angle) > 0 : angle = -0.15
                            if (my_add_error_angle - QR_B_angle) < 0 : angle =  0.15
                        if    (abs(my_add_error_angle - QR_B_angle) < 10) :
                            angle = 0
                            qr_moving_step = "목적지이동중"

                    if qr_moving_step == "목적지이동중":
                        if not(abs(QR_B_distance - my_add_error_distange) < 0.1) : speed = 0.3
                        if    (abs(QR_B_distance - my_add_error_distange) < 0.1) :
                            speed = 0
                            qr_moving_step = "출발지복귀중"
                        
                ## QR 코드가 "C"일 때, 지정해둔 C의 구역을 가기위해 지정 각도와 거리 이동
                if qr_target == "QR_C" :
                    if qr_moving_step == "각도변경중" : 
                        if not(abs(my_add_error_angle - QR_C_angle) < 10) : 
                            if (my_add_error_angle - QR_C_angle) > 0 : angle = -0.15
                            if (my_add_error_angle - QR_C_angle) < 0 : angle =  0.15
                        if    (abs(my_add_error_angle - QR_C_angle) < 10) :
                            angle = 0
                            qr_moving_step = "목적지이동중"

                    if qr_moving_step == "목적지이동중":
                        if not(abs(QR_C_distance - my_add_error_distange) < 0.1) : speed = 0.3
                        if    (abs(QR_C_distance - my_add_error_distange) < 0.1) :
                            speed = 0
                            qr_moving_step = "출발지복귀중"
                        
                ## QR A,B,C의 목표지점 까지 도착 후, QR 코드를 뺐을 때 각도 제어 및 출발지 복귀 
                if  qr_moving_step == "출발지복귀중" and qr_target == "QR_X":
                    run_direction = "후면"
                    if not(my_distance < 0.3) : speed = -0.3
                    if    (my_distance < 0.3) :
                        qr_moving_step = "행동초기화중1"

                if qr_moving_step == "행동초기화중1" :
                    if not(error_distance + 0.1 < 0.3) : speed = -0.3
                    if    (error_distance + 0.1 < 0.3) :
                        speed = 0
                        qr_moving_step = "행동초기화중2"
                    
                if qr_moving_step == "행동초기화중2" :
                    run_direction = "정면"
                    if not(my_add_error_angle == 0) : angle = -0.3
                    if    (my_add_error_angle == 0) : 
                        angle = 0
                        qr_moving_step = "QR대기중"
                        qr_moving = "QR대기중"
                        qr_target = "QR_X"


        # 속도와 각도 값을 지정 및 제한하여 지정 속도와 각도를 벗어나지 않도록 함
    
        if angle > 0.4 : angle = 0.4
        if angle < -0.4 : angle = -0.4
        if speed > 0.25 : speed = 0.25
        if speed < 0 and run_direction == "정면" : speed = 0

        print("--------------------------------")
        print("speed : ",speed)
        print("angle : ",angle)
        
        # print("X 위치값 : ", position_x)
        # print("Y 위치값 : ", position_y)


        # print("상하 각도 : ", pitch_y*(-1))
        print("mapping : ", mapping)
        # print("qr_target : ", qr_target)
        # print("qr_moving : ", qr_moving)
        # print("qr_moving_step : ", qr_moving_step)
        # print("barcode_data_product_QR :", barcode_data_product_QR)
        # print("run_direction :", run_direction)
        print("my_angle : ", my_angle)
        print("my_distance :", my_distance)
        print("my_add_error_angle : ", my_add_error_angle)
        print("my_add_error_distange :", my_add_error_distange)
        # print(orientation_x, orientation_y, orientation_z, orientation_w)
    except :
        speed = 0
        angle = 0

    talker()
        
        
def talker(): 

    # 로봇의 움직임 제어를 위한 Publishing
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = speed
    msg.angular.z = angle
    pub.publish(msg)

    
def listen():
    
    # L515 데이터 값 받기위한 Subsciber
    rospy.init_node("project3",anonymous=False)
    rospy.Subscriber("/camera/depth/image_rect_raw",Image,cam_lidar_read)
    rospy.Subscriber("/odom",Odometry,lidar_pose)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        listen()

    except rospy.ROSInterruptException:
        print('error')
        pass