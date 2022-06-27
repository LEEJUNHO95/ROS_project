#!/usr/bin/python3
# -*- coding: utf-8 -*-

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

from geometry_msgs.msg import Twist
import pyzbar.pyzbar as pyzbar
from datetime import datetime
import pyrealsense2 as rs
import numpy as np
import schedule
import rospy
import time
import cv2

frame_crop_x1 = 0
frame_crop_y1 = 120
frame_crop_x2 = 639
frame_crop_y2 = 479

minLineLength = 30
maxLineGap = 15

speed = 0
angle = 0
avr_x = 0
turn = -0.5

code_start = "start"
barcode_data_line_QR = []
text_0 = ""
text_1 = ""

## 동일한 qr코드 인식시 스피드 0 or 움직임
view_same_QR = 0
view_start_QR_and_no_product = 0

obstacle_view = 0

cap_0 = cv2.VideoCapture(2)
cap_1 = cv2.VideoCapture(4)

cap_1.set(cv2.CAP_PROP_FRAME_HEIGHT,180)
cap_1.set(cv2.CAP_PROP_FRAME_WIDTH,320)

def cam_0_read():
    global retval_0, frame_0, original, gray_line_0, gray_line_1
    retval_0, frame_0 = cap_0.read()
    original = frame_0
    gray_line_0 = cv2.cvtColor(frame_0, cv2.COLOR_BGR2GRAY)
    gray_line_1 = cv2.cvtColor(frame_0, cv2.COLOR_BGR2GRAY)

def cam_1_read():
    global retval_1, frame_1, gray_product_0
    retval_1, frame_1 = cap_1.read()
    gray_product_0 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)

def cam_0_use_line():
    global retval_0, frame_0, original, theta
    blurred = gray_line_0[frame_crop_y1:frame_crop_y2,frame_crop_x1:frame_crop_x2]
    blurred = cv2.boxFilter(blurred, ddepth=-1, ksize=(31,31))
    retval2 ,blurred = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
    edged = cv2.Canny(blurred, 85, 85)
    lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
    max_diff = 1000
    final_x = 0
    if ( lines is not None ):
        if ( lines is not None ):
            add_line = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(original,(x1+frame_crop_x1,y1+frame_crop_y1),(x2+frame_crop_x1,y2+frame_crop_y1),(0,255,0),3)
                mid_point = ( x1 + x2 ) / 2
                diff = abs((640/2) - mid_point)
                if ( max_diff > diff ) :
                    max_diff = diff
                    final_x = mid_point
                add_line = add_line + final_x
            average_x = add_line / len(lines)
        if ( int(average_x) != 0 ) :
            original = cv2.circle(original,(int(average_x),int((frame_crop_y1+frame_crop_y2)/2)),5,(0,0,255),-1)
            original = cv2.rectangle(original,(int(frame_crop_x1),int(frame_crop_y1)),(int(frame_crop_x2),int(frame_crop_y2)),(0,0,255),1)
        frame_0 = original
        theta = int(( int(average_x) - 320.0 ) / 640.0 * 100)
    if ( lines is None ):
        theta = -50

def cam_0_use_qrcode():
    global barcode_data_line_QR, barcode_type_line_QR
    decoded_line_QR = pyzbar.decode(gray_line_1)
    for _ in decoded_line_QR:
        x, y, w, h = _.rect
        barcode_data_line_QR = _.data.decode("utf-8")
        barcode_type_line_QR = _.type
        cv2.rectangle(frame_0, (x, y), (x + w, y + h), (0, 0, 255), 2)
        text_0 = '%s (%s)' % (barcode_data_line_QR, barcode_type_line_QR)
        cv2.putText(frame_0, text_0, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    if decoded_line_QR == [] :
        barcode_data_line_QR = "QR_X"

def cam_1_use_qrcode():
    global barcode_data_product_QR, barcode_type_product_QR
    decoded_product_QR = pyzbar.decode(gray_product_0)
    for _ in decoded_product_QR:
        a, b, c, d = _.rect
        barcode_data_product_QR = _.data.decode("utf-8")
        barcode_type_product_QR = _.type
        cv2.rectangle(frame_1, (a, b), (a + c, b + d), (0, 0, 255), 2)
        text_1 = '%s (%s)' % (barcode_data_product_QR, barcode_type_product_QR)
        cv2.putText(frame_1, text_1, (a, b), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    if decoded_product_QR == [] :
        barcode_data_product_QR = "QR_X"

def cam_lidar_read():
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, )
    pipeline.start(config)

def cam_lidar_use():
    global add_num, add_edge_num, add_edge_remain_num, image_all, left_view
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    coverage = [0]*32
    image_all = []
    add_num = 0
    add_edge_num = 0
    add_edge_remain_num = 0
    for y in range(240):
        for x in range(320):
            dist = depth.get_distance(x, y)
            if 0 < dist and dist < 1:
                coverage[x//10] += 1
        if y%20 is 19:
            line = ""
            for c in coverage:
                line += " 12345678"[c//25]
            coverage = [0]*32
            image_all.append(line)
            for a in range(1,32):
                if line[a] == " " : add_num = add_num
                else : add_num = add_num + int(line[a])
            for a in range(1,2):
                if line[a] == " " : add_edge_num = add_edge_num
                else : add_edge_num = add_edge_num + int(line[a])
            for a in range(3,32):
                if line[a] == " " : add_edge_remain_num = add_edge_remain_num
                else : add_edge_remain_num = add_edge_remain_num + int(line[a])

def speed_and_angle_make():
    global angle, speed
    angle = round((-theta) * (0.012), 2)
    speed = 0.3 - abs(angle * 0.2)

def speed_and_angle_turn():
    global angle, speed
    speed = 0      
    angle = turn

def speed_and_angle_main():
    global angle, speed, barcode_data_product_QR, barcode_data_line_QR, turn, obstacle_view, view_same_QR, view_start_QR_and_no_product
   
    if barcode_data_line_QR == "right_turn"  : turn = -0.5
    if barcode_data_line_QR == "left_turn" : turn = 0.5

    if view_same_QR == 0 and view_start_QR_and_no_product == 0 :
        if obstacle_view == 0 :
            if theta != -50:
                if add_num <= 10                                                                                           : speed_and_angle_make()
                if add_num <= 10 and barcode_data_product_QR != "QR_X" and barcode_data_product_QR == barcode_data_line_QR : view_same_QR = 1
                if add_num <= 10 and barcode_data_product_QR == "QR_X" and barcode_data_line_QR    == "start"              : view_start_QR_and_no_product = 1
                if add_num >  10 : 
                    speed = 0
                    obstacle_view = 1
            if theta == -50 : speed_and_angle_turn()
                
        if obstacle_view == 1 :
            if add_num != 0 :  
                if add_edge_num        > 0 and add_edge_remain_num == 0 : angle = -0.4
                if add_edge_remain_num > 0 and add_edge_num == 0 : angle = -0.4
                if add_edge_remain_num > 0 and add_edge_num > 0 : angle = -0.4
                speed = 0.2
            if add_num == 0                  : angle = 0.4
            if theta != -50 and add_num == 0 : obstacle_view = 0

    if view_same_QR == 1 or view_start_QR_and_no_product == 1: 
        speed = 0
        angle = 0
        if view_same_QR == 1                 and barcode_data_product_QR == "QR_X" : view_same_QR = 0
        if view_start_QR_and_no_product == 1 and barcode_data_product_QR != "QR_X" : view_start_QR_and_no_product = 0

def talker():
    global speed, angle
    rospy.init_node("line_qr_sensor")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()

    cam_lidar_read()

    while not rospy.is_shutdown():

        msg.linear.x = speed
        msg.angular.z = angle
        pub.publish(msg)

        cam_0_read()
        cam_0_use_line()
        cam_0_use_qrcode()

        cam_1_read()
        cam_1_use_qrcode()

        cam_lidar_use()

        speed_and_angle_main()

        for y in range(12):
            print(image_all[y])
        print(add_num)
        print("장애물 : ", obstacle_view)
        print("세타값 : ", theta)
        print("턴값 : ", turn)
        cv2.imshow('frame_0', frame_0)
        cv2.imshow('frame_1', frame_1)
        
        key = cv2.waitKey(25)
        if key == 27: #ESC
            break


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass