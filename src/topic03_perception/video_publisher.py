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

if __name__ == "__main__":
    bridge = CvBridge()
    
    # Declare video path
    video_path = "/dev/video0"  # webcam
    # video_path = "/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/video/tennis-ball-video.mp4"

    video = import_video(video_path)
    
    rospy.init_node("video_publisher")
    video_publisher = rospy.Publisher("/video", Image)
    
    rospy.loginfo("Video publisher node started")
    
    while(True):
        ret, frame = video.read()
        
        try:
            print(ret)
            cv2.imshow("Test Publisher", frame)
            video_publisher.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            # bridge.cv2_to_imgmsg(frame, 'bgr8')
            
        except CvBridgeError as e:
            print(e)
        
        if cv2.waitKey(frame_rate(10)) & 0xFF==ord('q'): 
            break
        
    video.release()
    cv2.destroyAllWindows()