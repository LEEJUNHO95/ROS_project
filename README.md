# ROS_project - project1. line_detect part & project2. SLAM_move part
Driving Video Link <line_detect> <<https://www.youtube.com/watch?v=QiVPxZddGn0>>

Driving Video Link <SLAM_MOVE> <<https://www.youtube.com/watch?v=w1bOpxk9GYY&t=1s>>

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

     Reference Site
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
        
**If you have any questions about code, please leave a comment on YouTube or leave a comment on Git**
        


