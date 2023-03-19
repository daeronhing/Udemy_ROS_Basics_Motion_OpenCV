#!/usr/bin/env python3

#import numpy: the data structure that will handle an image
import numpy as np

#import openCV
import cv2

import os

root_folder = "/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/images/"
image_name = "flower"
file_extension = ".jpg"

img_path = os.path.join(root_folder,image_name+file_extension)

print ('read an image from file')
# img = cv2.imread("/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/images/flower.jpg")
img = cv2.imread(img_path)

print ('create a window holder for the image')
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)

print ('display the image')
cv2.imshow("Image",img)

print ('press a key inside the image to make a copy')
cv2.waitKey(0)

# print ('image copied to folder images/copy/')
# img_path = os.path.join(root_folder,"copy",image_name+"-copy"+file_extension)
# cv2.imwrite("~/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/images/copy/"+image_name+"-copy.jpg",img)
# cv2.imwrite(img_path,img)
