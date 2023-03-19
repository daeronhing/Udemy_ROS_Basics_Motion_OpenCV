#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <stdio.h>

using namespace std;
using namespace cv;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "video_publisher_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/video", 1);

    // Import video
    VideoCapture video("/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/video/tennis-ball-video.mp4");

    Mat frame;
    while(true) {
        video >> frame;

        // Publish the image
        sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        image_pub.publish(imgmsg);
        ros::spinOnce();

        if(waitKey(30) >= 0) break;
    }

    ROS_INFO("Finish");
    return 0;
}