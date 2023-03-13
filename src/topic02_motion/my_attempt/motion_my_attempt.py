#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def callback_pose(pose_msg):
    global x,y, theta
    x = pose_msg.x
    y = pose_msg.y
    theta = pose_msg.theta

def move(topic, distance, speed, is_forward=True):
    velocity_message = Twist()
    velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)
    
    if is_forward:
        velocity_message.linear.x = speed
    else:
        velocity_message.linear.x = -speed
    
    global x,y
    x0 = x
    y0 = y
    distance_travelled = 0
    rate = rospy.Rate(10)
    
    while(distance_travelled < distance):
        velocity_publisher.publish(velocity_message)
        rate.sleep()
        distance_travelled = math.sqrt((x-x0)**2 + (y-y0)**2)
        rospy.loginfo("Travelled:%f"%distance_travelled)
    
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Reached")
    return "Reached"

def rotate(topic, angle_of_rotation, angular_speed, anti_clockwise=True):
    angular_velocity_message = Twist()
    angular_velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)
    
    if anti_clockwise:
        angular_velocity_message.angular.z = angular_speed
    else:
        angular_velocity_message.angular.z = -angular_speed
    
    global theta
    theta0 = theta
    rotated_angle = 0
    rate = rospy.Rate(10)
    
    while(abs(rotated_angle) < abs(angle_of_rotation)):
        angular_velocity_publisher.publish(angular_velocity_message)
        rate.sleep()
        rotated_angle = theta - theta0
        rospy.loginfo("Target:%s, rotated:%s"%(angle_of_rotation,theta))
    
    angular_velocity_message.angular.z = 0
    angular_velocity_publisher.publish(angular_velocity_message)
    rospy.loginfo("Finished")
    return "Finished"
    

if __name__ == "__main__":
    rospy.init_node("motion_my_attempt")
    
    pose_topic = "/turtle1/pose"
    pose_subscriber = rospy.Subscriber(pose_topic, Pose, callback_pose)
    
    rospy.sleep(0.1)
    
    velocity_topic = "/turtle1/cmd_vel"
    rotate(velocity_topic, 1, 0.1)
    move(velocity_topic, 3, 1)