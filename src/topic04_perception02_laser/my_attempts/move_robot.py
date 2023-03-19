#!/usr/bin/env python3

#########################
# 1. To move the robot, publish to /cmd_vel topic
# 2. Transformation is stored in /tf topic
# 3. Check obstacle distance by subscribing to /camera/depth/points
#########################

import rospy
import math
from math import sqrt
from geometry_msgs.msg import Twist # cmd_vel msg type
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

DISTANCE_THRESHOLD = 0.01
K_LINEAR = 0.85

def odomCallback(odom_msg):
    global x,y
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    q = odom_msg.pose.pose.orientation
    
    # roll, pitch, yaw = euler_from_quaternion(q)    

def scanCallback(scan_msg):
    rad = math.radians(5)
    increment = scan_msg.angle_increment
    index = (int)(rad/increment)
    
    global left_avg_range, right_avg_range
    ranges = [x for x in scan_msg.ranges if not math.isnan(x)]
    left_avg_range = average_range(ranges, 0, index+1)
    right_avg_range = average_range(ranges, -index, -1)

def average_range(ranges, start_index, end_index):
    slice_of_array = ranges[start_index:end_index]
    return ( sum(slice_of_array) / float(len(slice_of_array) ))

def param_initialization(param_name, value):
    if not rospy.has_param(param_name):
        rospy.set_param(param_name, value)
    rospy.get_param(param_name, value)

def set_linear_velocity(linear_speed):   
    cmd_vel_msg.linear.x = linear_speed
    cmd_vel_pub.publish(cmd_vel_msg)

def set_angular_velocity(angular_velocity):
    cmd_vel_msg.angular.z = angular_velocity
    cmd_vel_pub.publish(cmd_vel_msg)

def move(distance):
    rate = rospy.Rate(10)
    rospy.loginfo("Moving forward...")
    
    global x,y
    x0,y0 = x,y # Initial position
    while(True):
        distance_travelled = sqrt((x-x0)**2 + (y-y0)**2)
        gap = distance - distance_travelled
        
        global left_avg_range, right_avg_range, obstacle_flag
        if(gap < DISTANCE_THRESHOLD):
            rospy.loginfo("Reached")
            break

        elif(left_avg_range < 0.6 or right_avg_range < 0.6):
            rospy.loginfo("Obstacle ahead...")
            obstacle_flag = True
            break
        
        linear_speed = K_LINEAR * gap
        set_linear_velocity(linear_speed)

        rate.sleep()
    
    set_linear_velocity(0)
    
def avoid_obstacle():
    rate = rospy.Rate(10)
    angular_speed = 0.3
    
    global left_avg_range, right_avg_range, obstacle_flag
    while(True):
        print("\nleft:%f, right:%f"%(left_avg_range,right_avg_range))
        if(left_avg_range > 2.5 and right_avg_range > 2.5):
            obstacle_flag = False
            rospy.loginfo("Dodged obstacle")
            break
            
        elif(left_avg_range < right_avg_range):     # obstacle on the left, turn right
            angular_velocity = -angular_speed
            
        else:                                       # obstacle on the right, turn left
            angular_velocity = angular_speed
    
        set_angular_velocity(angular_velocity)
        
        rate.sleep()

    set_angular_velocity(0)

if __name__ == "__main__":
    rospy.init_node("assignment")
    param_initialization("/distance_threshold", DISTANCE_THRESHOLD)
    param_initialization("/K_LINEAR", K_LINEAR)
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    laser_scan = rospy.Subscriber("/scan", LaserScan, scanCallback)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    rospy.loginfo("Initializing....")
    rospy.wait_for_message("/odom", Odometry)
    rospy.wait_for_message("/scan", LaserScan)
    rospy.loginfo("Node running")
    
    cmd_vel_msg = Twist()
    # rospy.spin()
    global left_avg_range, right_avg_range, obstacle_flag
    if( left_avg_range > 0.6 and right_avg_range > 0.6 ): obstacle_flag = False
    else: obstacle_flag = True
    while(True):
        if obstacle_flag:
            avoid_obstacle()
        move(0.1)