# ROS_project - project1. line_detect part & project2. SLAM_move part
**Driving Video Link <line_detect> <<https://www.youtube.com/watch?v=QiVPxZddGn0>>**

**Driving Video Link <SLAM_MOVE> <<https://www.youtube.com/watch?v=w1bOpxk9GYY&t=1s>>**

## Common Hardware Development Environments
● scout MINI 
<img src = "https://user-images.githubusercontent.com/98440628/176610886-aa75f777-31e0-460b-a51e-7033cd9d9cd2.png" width="20%" height="20%">

● Intel L515 RealSense 
<img src = "https://user-images.githubusercontent.com/98440628/176611606-8a1c7cfc-bb6c-4fce-a34e-f3d8a8edf493.png" width="20%" height="20%">

● Logitech C920 PRO HD WEBCAM
<img src = "https://user-images.githubusercontent.com/98440628/176611962-189edcf8-85bb-445b-8568-04c5e6871cb7.png" width="20%" height="20%">

## Common Software Development Environments
Ubuntu 18.04 Installation

1. ROS Installation
2. Reference Site 
<<https://wiki.ros.org/melodic/Installation/Ubuntu>> 1.2 ~ 1.6.1 Installation

3. scout_mini SDK Installation
Reference Site 
<<https://github.com/agilexrobotics/ugv_sdk>>

4. scout_mini Package installation
Reference Site
<<https://github.com/agilexrobotics/scout_ros>>


    ## Please keep in mind

        If you installed SDK & package and Scout_MIni doesn't work, please write it down on GIT Issue. I'll give you a solution

5. intel Realsense L515 SDK installation 
     **Ubuntu 18.04**
         
         $ sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo bionic main" -u
         $ sudo apt-get install librealsense2-dkms
         $ sudo apt-get install librealsense2-utils
         $ sudo apt-get install librealsense2-dev
         $ sudo apt-get install librealsense2-dbg
         
         ## test
         $ realsense-viewer
6. **If you are going to use SLAM, refer to this GitHub link. I referred to this package**

     **Reference Site**
        <<https://github.com/wh200720041/ssl_slam>>

## Project1. line_detect

**library used**

        1. install pyzbar
        2. install pyrealsense2
        3. install numpy
        4. install cv2
        5. install time
        6. install datetime

**massage used**

        from geometry_msgs.msg import Twist ## Massage for scout MINI movement

## Video for Project 1


● **Modified scout MINI for rail recognition**


            # Parts Used #
            ● scout MINI : 1
            ● Logitech C920 PRO HD WEBCAM : 2
            ● Intel L515 3D Rider sensor : 1

<img src = "https://user-images.githubusercontent.com/98440628/176838379-9cd06580-97e5-4a1a-914f-cd0a0eb70aae.jpg" width="30%" height="30%">



● **Video with rail recognition algorithm // Rail Recognition**

<img src = "https://user-images.githubusercontent.com/98440628/176839815-e7fe8361-2d26-4a92-a167-10a97924fb96.gif" width="70%" height="70%">
            
            
            # Explanation #
            1) cvtColor 함수 : 영상을 gray 스케일로 변환
            2) boxFilter 함수 : 노이즈를 제거하기 위해서 사용
            3) Threshold 함수 : 영상을 임계값 기준으로 흑/백 분류
            4) Canny 함수 : 경계선 (선) 검출
            5) HoughLinesP 함수 : 직선으로 검출된 Line의 양 끝점을 (x1, y1, x2, y2) 형식으로 data 추출
            6) 검출된 각 Line의 x 값의 평균값을 활용하여 무인이동체가 레일을 따라 움직이도록 구현


● **Video with QR code Recognition**

<img src = "https://user-images.githubusercontent.com/98440628/176840866-682a4258-69f9-4bb1-9f42-f815f0e5d958.gif" width="50%" height="50%">

            
            # Explanation - Use pyzbar library #
            1) QR코드 생성 ( Start, A, B, C )
            2) 웹캠1에서 Start QR을 인식하면, 무인이동체 정지
            3) 웹캠2에서 A or B or C QR을 인식하면, 인식된 QR위치로 이동 후 정지
            4) 인식된 QR의 위치로 도착 후, 웹캠2의 QR을 제거하면 주행 재개
            
            
● **Video with Obstacles recognition and avoidance**
            
<img src = "https://user-images.githubusercontent.com/98440628/176846391-58269628-065d-43a4-bb3f-1d320e5e763f.gif" width="60%" height="60%">            
            
            
            # Explanation - Use pyrealsense2 library #
            1) 3D 라이다카메라의 depth값을 사용
            2) depth값에 따라 영상 각 픽셀에 “ ,1,2,3,4,5,6,7,8” 로 표시되도록 알고리즘 설계
            3) 알고리즘에 의해 표시된 값으로 장애물 유무 판단
            4) 장애물을 회피하도록 무인이동체의 속도와 각도를 제어

● **Project1 - Line Detect __Full Version Video__**

**<<https://www.youtube.com/watch?v=QiVPxZddGn0>>**

## Project2. SLAM_MOVE

**library used**

        1. install pyzbar
        2. install pyrealsense2
        3. install numpy
        4. install cv2
        5. install realcamera
        6. install math

**massage used**

        from geometry_msgs.msg import Twist ## Massage for scout MINI movement
        from nav_msgs.msg import Odometry ## Massage for scout MINI position
        

## Video for Project 2


● **Modified scout MINI for SLAM recognition**

            # Parts Used #
            ● scout MINI : 1
            ● Logitech C920 PRO HD WEBCAM : 1
            ● Intel L515 3D Rider sensor : 1


<img src = "https://user-images.githubusercontent.com/98440628/176847640-386501b7-38f3-4a0c-a022-2a44d38b5e3f.jpg" width="30%" height="30%">



 ● **Video of my office that Use SSL_SLAM // Reference Site <<https://github.com/wh200720041/ssl_slam>>** 
 
 
 <img src = "https://user-images.githubusercontent.com/98440628/176849576-33b05a77-2e20-4ba7-bc55-86cc73ce1d66.gif" width="60%" height="60%">
        


● **Convert raw_data of L515_Depth to sorted list**



 <img src = "https://user-images.githubusercontent.com/98440628/176851020-66ad8c1f-a340-426b-88af-bcd489242b1f.gif" width="60%" height="60%">
 
 
            # Explanation for this video #
            1) 3D 라이다카메라의 depth은 307200개의 raw_data를 가지고 있었다.
            2) 이 depth 값을 쓰려면 raw_data를 정렬된 데이터로 바꿔야 한다.
            3) bytearray 함수를 써서 raw_data를 변환하고 numpy를 사용하여 이것을 빠른 속도로 계산하여 정렬된 리스트로 변환시킨다.


● **Project2 - SLAM_MOVE __Full Version Video__**

**<<https://www.youtube.com/watch?v=w1bOpxk9GYY&t=1s>>**


**If you have any questions about code, please leave a comment on YouTube or leave a comment on Git**
        


