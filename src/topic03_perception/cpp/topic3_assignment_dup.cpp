#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool show_original_bgr = true;
bool show_hsv = true;
bool show_gray = false;

double framerate(int hz) {
    return 1000/hz;
}

int main (int argc, char** argv)
{
    // Import video
    VideoCapture video_capture("/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/video/tennis-ball-video.mp4");

    Mat frame, hsv, mask;
    while(true) {
        video_capture >> frame;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // Create mask
        inRange(hsv, Scalar(30, 100, 125), Scalar(45, 255, 255), mask);

        // Find contour
        vector<vector<Point>> contours;
        vector<Vec4i> heirarchy;
        findContours(mask, contours, heirarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Draw contour
        Point2f center;
        float radius;
        for(int i=0;i<contours.size(); i++) {
            double area = contourArea(contours[i]);
            double perimeter = arcLength(contours[i], true);
            if(area>100) {
                minEnclosingCircle(contours[i],center,radius);
                drawContours(frame, contours, -1, Scalar(0,0,255));
                circle(frame, center, (int)radius, Scalar(0,0,0));
            }
        }

        namedWindow("Video", WINDOW_NORMAL);
        imshow("Video", frame);

        if(waitKey(30) >= 0) break;
    }

    destroyAllWindows();
    return 0;

    // // Import image
    // Mat image = imread("/home/daeronhing/Desktop/ROS_OpenCV/catkin_ws/src/ros_essentials_cpp-ros-noetic/src/topic03_perception/images/tennisball01.jpg", IMREAD_COLOR);


    // if(!image.data) {
    //     cout << "Could not open the image file" << std::endl;
    //     return -1;
    // }

    // // Convert into gray_scale and hsv
    // Mat hsv, gray;
    // cvtColor(image, hsv, COLOR_BGR2HSV);
    // cvtColor(image, gray, COLOR_BGR2GRAY);


    // // Create mask
    // Mat mask;
    // inRange(hsv, Scalar(30, 100, 125), Scalar(45, 255, 255), mask);

    // namedWindow("Mask", WINDOW_NORMAL);
    // imshow("Mask", mask);

    // // Find contour
    // vector<vector<Point>> contours;
    // vector<Vec4i> heirarchy;
    // findContours(mask, contours, heirarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // cout << "Size is: " << contours.size() << endl;

    // double area, perimeter;
    // int cx, cy;
    // Point2f center;
    // float radius;
    // // Draw contour
    // for(int i=0; i<contours.size();i++) {
    //     area = contourArea(contours[i]);
    //     perimeter = arcLength(contours[i], true);
    //     minEnclosingCircle(contours[i], center, radius);
    //     if(area > 100) {
    //         drawContours(image, contours, -1, Scalar(0,0,255));
    //         circle(image, center, (int)radius, Scalar(0,0,0));
    //     }
    // }

    // // cout << "Area:" << area << " Perimeter:" << perimeter << " Center:" << center << " radius:" << radius << endl;

    // // Display image
    // if(show_original_bgr) {
    //     namedWindow("BGR image", WINDOW_NORMAL);
    //     imshow("BGR image", image);
    // }

    // if(show_hsv) {
    //     namedWindow("HSV", WINDOW_NORMAL);
    //     imshow("HSV", hsv);
    // }

    // if(show_gray) {
    //     namedWindow("Gray", WINDOW_NORMAL);
    //     imshow("Gray", gray);
    // }
    
    // waitKey(0);
    // destroyAllWindows();
    // return 0;
}
