#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

using namespace std;

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("test", 1);
    ros::Subscriber sub = nh.subscribe("test", 1, callback);
    cout << "Waiting for message" << endl;

    // boost::shared_ptr<std_msgs::String> test;
    ros::topic::waitForMessage<std_msgs::String>("test");
    cout << "Message received" << endl;

    std_msgs::String pub_msg;
    ros::Rate rate(1);

    pub_msg.data = "Testing";
    cout << "Publishing" << endl;
    while(ros::ok()) {
        pub.publish(pub_msg);

        ros::spinOnce();
        rate.sleep();
    }
}