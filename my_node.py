#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
import rosbag
import cv2
from cv_bridge import CvBridge
from duckietown_utils import rgb_from_ros

print("hello")

bag = rosbag.Bag("/duckietown/my-ros-program/dec_17/luthor_2019-12-17-23-57-40.bag")

bridge = CvBridge()
count = 0

time = []

#f = open("/duckietown/my-ros-program/dec_17/luthor_2019-12-17-23-32-14_image_time.txt", "a")
for topic,msg, t in bag.read_messages(topics = ["/luthor/camera_node/image/compressed"]):
    #print(msg.vel_left)
    #["/luthor/camera_node/image/compressed"]):
    #["/a313/camera_node/image/compressed"]):
    #["/luthor/wheels_driver_node/wheels_cmd"]):

    #print("*******")
    #print(msg.header)
    #print(msg.header.stamp)
    #print("^^^^^")
    #time_msg = msg.header.stamp
    #time.append(float(str(time_msg)))

    #f.write(str(msg.vel_left) +" " + str(msg.vel_right) + " " + str(msg.header.stamp) + " \n")
    #f.write(str(msg.header.stamp) +" \n")

    #print("***********************************")

    #cv_img = bridge.imgmsg_to_cv2(msg,desired_encoding = "passthrough")

    #print(count)
    cv_img = rgb_from_ros(msg)
    cv2.imwrite(os.path.join("/duckietown/my-ros-program/dec_17/images_luthor_2019-12-17-23-57-40", "frame%06i.png" %count), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
    count +=1
    #time.append(t)

    #print(t)

bag.close()

#42f.close()

#print(type(time[1]))
#print(int(str(time[1])))
#print(time[1])


print("yolo")