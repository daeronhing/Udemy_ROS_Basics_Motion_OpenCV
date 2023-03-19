#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;

double framerate(int hz) {
    return 1000/hz;
}

void imageCallback(const sensor_msgs::ImageConstPtr& imgmsg) {
    Mat frame, hsv, mask;
    frame = cv_bridge::toCvCopy(imgmsg, "bgr8")->image;
    
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    inRange(hsv, Scalar(30,100,125), Scalar(45,255,255), mask);

    vector<vector<Point>> contours;
    vector<Vec4i> heirarchy;
    findContours(mask, contours, heirarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Point2f center;
    float radius;
    for(int i=0;i<contours.size();i++) {
        double area = contourArea(contours[i]);
        double perimeter = arcLength(contours[i], true);
        if(area>100) {
            minEnclosingCircle(contours[i],center,radius);
            drawContours(frame,contours,-1,Scalar(0,0,255));
            circle(frame,center,(int)radius,Scalar(0,0,0));
        }
    }

    namedWindow("Subscriber", WINDOW_NORMAL);
    imshow("Subscriber", frame);
    waitKey(30);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "assignment");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber video_sub = it.subscribe("/video", 1, imageCallback);

    ros::spin();
    destroyAllWindows();
    return 0;
}
