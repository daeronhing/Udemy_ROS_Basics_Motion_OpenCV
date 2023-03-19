#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def import_video(src):
    video = cv2.VideoCapture(src)
    return video

def frame_rate(hz):
    return (int)(1000/hz)

def read_bgr_image(src, show_image=False):
    bgr_image = cv2.imread(src, cv2.IMREAD_COLOR)
    if(show_image):
        cv2.namedWindow("BGR_image", cv2.WINDOW_NORMAL)
        cv2.imshow("BGR_image",bgr_image)
    return bgr_image

def filter_color(src, lower_bound, upper_bound, show_image=False):
    # Convert image into HSV color space
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound) 
    if(show_image):
        cv2.namedWindow("HSV", cv2.WINDOW_NORMAL)
        cv2.imshow("HSV", hsv)
    
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        cv2.imshow("Mask", mask)
        
    return mask

def getContours(binary_image):
    contours, hierarchy = cv2.findContours(binary_image, 
                                           cv2.RETR_EXTERNAL, 
                                           cv2.CHAIN_APPROX_SIMPLE)
    return contours


def drawContours(bgr_image, contours, show_image=False):
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        center, radius = cv2.minEnclosingCircle(contour)
        center = ((int)(center[0]), (int)(center[1]))
        if(area>100):
            cv2.drawContours(bgr_image, [contour], -1, (0,0,255))
            cx, cy = get_contour_center(contour)
            cv2.circle(bgr_image, center, (int)(radius), (0,0,0))
            cv2.circle(bgr_image, (cx,cy), (int)(radius), (255,0,0))
    
    if(show_image):
        cv2.namedWindow("Edited BGR image", cv2.WINDOW_NORMAL)
        cv2.imshow("Edited BGR image", bgr_image)

    return bgr_image

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def template():
    # Declare image path
    img_folder_path = "/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/images"
    image_name = "tennisball03"
    file_extension = ".jpg"
    img_path = img_folder_path + '/' + image_name + file_extension

    # Read BGR image
    image_bgr = read_bgr_image(img_path)
    
    # # Extract info from image
    # height, width, channels = image_bgr.shape

    # Filter color
    # lower_bound = (0, 100, 100)   # Good for 1, 4
    # upper_bound = (50, 255, 255)  # Good for 1, 4
    # lower_bound = (30, 100, 125)    # Good for 1, 2, 4, 5
    # upper_bound = (45, 255, 255)    # Good for 1, 2, 4, 5
    lower_bound = (30, 100, 125)    # Good for 1, 2, 4, 5
    upper_bound = (45, 255, 255)    # Good for 1, 2, 4, 5
    binary_image_mask = filter_color(image_bgr, lower_bound, upper_bound, True)
    
    # # Show mask image
    # cv2.imshow("Mask", binary_image_mask)
    
    # Draw contours
    contours = getContours(binary_image_mask)
    edited_image = drawContours(image_bgr, contours)
    
    cv2.imshow("Edited_image", edited_image)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
def ball_tracking_by_frame(frame, show_video=True):
    lower_bound = (30, 100, 125)    # Good for 1, 2, 4, 5
    upper_bound = (45, 255, 255)    # Good for 1, 2, 4, 5
    binary_image_mask = filter_color(frame, lower_bound, upper_bound)
    
    # # Show mask image
    # cv2.imshow("Mask", binary_image_mask)
    
    # Draw contours
    contours = getContours(binary_image_mask)
    drawContours(frame, contours, show_video)
    
def assigment():
    # Declare video path
    video_folder_path = "/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/video"
    video_name = "tennis-ball-video.mp4"
    video_path =video_folder_path + '/' + video_name

    video = import_video(video_path)
    
    while(True):
        ret, frame = video.read()
        frame = cv2.resize(frame, (0,0), fx=0.5,fy=0.5)
        ball_tracking_by_frame(frame)
        
        if cv2.waitKey(frame_rate(60)) & 0xFF==ord('q'): 
            break
    
    video.release()
    cv2.destroyAllWindows()

def imageCallback(ros_image):
    rospy.loginfo("Received")
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    # cv2.namedWindow("Test Subscriber", cv2.WINDOW_NORMAL)
    # cv2.imshow("Test Subscriber", frame)
    ball_tracking_by_frame(frame)
    cv2.waitKey(frame_rate(60))
    
if __name__ == "__main__":
    rospy.init_node("assignment")
    bridge = CvBridge()
    
    video_subscriber = rospy.Subscriber("/video", Image, imageCallback)
    rospy.spin()