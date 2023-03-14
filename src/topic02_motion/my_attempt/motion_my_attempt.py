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
        rospy.loginfo("Target:%f, Travelled:%f"%(distance,distance_travelled))
    
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Reached")
    return "Reached"

def rotate(topic, angle_rad, angular_speed_rad, anti_clockwise=True):
    angular_velocity_message = Twist()
    angular_velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)
    
    if anti_clockwise:
        angular_velocity_message.angular.z = angular_speed_rad
    else:
        angular_velocity_message.angular.z = -angular_speed_rad
    
    global theta
    target_angle = (theta + angle_rad)%(2*math.pi)
    if(target_angle > math.pi):
        target_angle -= 2*math.pi
    elif(target_angle < -math.pi):
        target_angle += 2*math.pi

    rate = rospy.Rate(10)
    
    while(abs(theta-target_angle)>0.01):
        angular_velocity_publisher.publish(angular_velocity_message)
        rospy.loginfo("Relative:%s, target:%s, current:%s"%(angle_rad, target_angle,theta))
        
        rate.sleep()
        
    angular_velocity_message.angular.z = 0
    angular_velocity_publisher.publish(angular_velocity_message)
    rospy.loginfo("Rotation Finished")
    return "Finished"
    
def go_to(topic, x_goal, y_goal):
    global x,y,theta
    angle_of_rotation = math.atan2(y_goal-y, x_goal-x) - theta

    angular_speed = 0.2
    if(angle_of_rotation > 0): 
        anti_clockwise = True
    else:
        anti_clockwise = False

    rotate(topic, angle_of_rotation, angular_speed, anti_clockwise)
    
    distance = math.sqrt((x_goal-x)**2 + (y_goal-y)**2)
    
    speed = 1
    move(topic, distance, speed, True)
    
    rospy.loginfo("Destination Reached")
    return "Reached"

if __name__ == "__main__":
    rospy.init_node("motion_my_attempt")
    
    pose_topic = "/turtle1/pose"
    pose_subscriber = rospy.Subscriber(pose_topic, Pose, callback_pose)
    
    rospy.sleep(0.1)
    
    velocity_topic = "/turtle1/cmd_vel"
    # rotate(velocity_topic, 1, 1)
    # move(velocity_topic, 3, 1)
    go_to(velocity_topic, 0,0)