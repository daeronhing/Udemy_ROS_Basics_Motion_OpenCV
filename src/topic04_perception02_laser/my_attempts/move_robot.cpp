#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

const double PI 3.141592653589793238;

void odomCallback(nav_msgs::Odometry odom_msg);
void scanCallback(sensor_msgs::LaserScan scan_msg);
double average_range();


double current_x;
double current_y; 
double current_yaw;

double left_avg_range;
double right_avg_range;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "assignment");
    ros::NodeHandle nh;

    ros::Subscriber odomSubscriber = nh.subscribe("/odom", 1, odomCallback);
    ros::Subscriber scanSubscriber = nh.subscribe("/scan", 1, scanCallback);

    ros::spin();
}

void odomCallback(nav_msgs::Odometry odom_msg) {
    current_x = odom_msg.pose.pose.position.x;
    current_y = odom_msg.pose.pose.position.y;

    // cout << "Odom callback, x: " << current_x << " y: " << "current_y" << endl;
}

void scanCallback(sensor_msgs::LaserScan scan_msg) {
    double rad = 5 / 180 * PI;
    double increment = scan_msg.angle_increment;
    int index = rad/increment;

    for(int i=0; )
    

}

def scanCallback(scan_msg):
    rad = math.radians(5)
    increment = scan_msg.angle_increment
    index = (int)(rad/increment)
    
    global left_avg_range, right_avg_range
    ranges = [x for x in scan_msg.ranges if not math.isnan(x)]
    left_avg_range = average_range(ranges, 0, index+1)
    right_avg_range = average_range(ranges, -index, -1)