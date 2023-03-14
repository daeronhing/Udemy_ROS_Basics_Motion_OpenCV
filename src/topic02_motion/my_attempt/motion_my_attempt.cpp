#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

ros::Subscriber pose_subscriber;
ros::Publisher velocity_publisher;

turtlesim::Pose current_pose;
geometry_msgs::Twist vel_msg;

const float Kp_linear = 1;
const float Kp_angular = 1;
const double PI = 3.14159265359;

void poseCallback(turtlesim::Pose pose_msg);
void move(double distance, bool is_forward);
void rotate(double angle_rad, bool ccw);
void go_to(double x, double y);

int main (int argc, char** argv)
{
    ros::init(argc, argv, "motion_my_attempt");
    ros::NodeHandle nh;
    
    pose_subscriber = nh.subscribe("/turtle1/pose", 10, poseCallback);
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Duration(1).sleep();
    ros::spinOnce();

    ROS_INFO("Node motion_my_attempt up running");

    move(4, true);
    rotate(3.14, true);
    move(4, true);
    rotate(1, true);
    move(1, true);
    rotate(-1, false);
    move(1, true);

    go_to(5.544445, 5.544445);
}

void poseCallback(turtlesim::Pose pose_msg) {
    current_pose.x = pose_msg.x;
    current_pose.y = pose_msg.y;
    current_pose.theta = pose_msg.theta;
}

void move(double target_distance, bool is_forward) {
    float x0 = current_pose.x;
    float y0 = current_pose.y;
    double distance_travelled = 0.0;
    double gap = target_distance - distance_travelled;
    int forward_flag = 1;
    ros::Rate rate(10);

    if(!is_forward) forward_flag = -1;

    while(true)
    {
        vel_msg.linear.x = Kp_linear * gap * forward_flag;
        velocity_publisher.publish(vel_msg);

        // ROS_INFO("x0:%f, x:%f, y0:%f, y:%f",x0, current_pose.x, y0, current_pose.y);
        distance_travelled = sqrt(pow(current_pose.x-x0,2) + pow(current_pose.y-y0,2));
        gap = target_distance - distance_travelled;
        
        ROS_INFO("Travelled:%f, gap:%f", distance_travelled, gap);
        if(abs(gap) < 0.05) break;

        ros::spinOnce();
        rate.sleep();
    }

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Reached");
}

void rotate(double angle_rad, bool ccw) {
    double target_angle = current_pose.theta + angle_rad;
    ROS_INFO("Target_angle:%f",target_angle);
    int ccw_flag = 1;
    ros::Rate rate(100);

    if(!ccw) ccw_flag = -1;

    if(target_angle > PI) target_angle = target_angle - (2*PI);
    else if(target_angle < -PI) target_angle = 2*PI + target_angle;

    double gap = target_angle - current_pose.theta;
    while(true)
    {
        ROS_INFO("Gap:%f, current:%f",gap,current_pose.theta);
        vel_msg.angular.z = Kp_angular * abs(gap) * ccw_flag;
        velocity_publisher.publish(vel_msg);

        gap = target_angle - current_pose.theta;
        if(abs(gap) < 0.01) break;

        ros::spinOnce();
        rate.sleep();
    }

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Finished");
}

void go_to(double x, double y) {
    double delta_y = y-current_pose.y;
    double delta_x = x-current_pose.x;
    bool ccw;

    double delta_theta = atan2(delta_y,delta_x) - current_pose.theta;
    if(delta_theta>=0 && delta_theta<=PI) ccw=true;
    else ccw=false;
    rotate(delta_theta, ccw);

    double distance = sqrt(pow(delta_y,2)+pow(delta_x,2));
    move(distance, true);

    ROS_INFO("Reached Destination");
}