# ROS Notes for OpenCV (Python, C++)

This note contains basic usage of OpenCV in Python and C++, and how to use together with ROS

# Content
- [Image Input Output](#image-input-output)
    - [Reading image](#reading-image)
    - [Display image](#display-image)
    - [Save image](#save-image)
- [Image Structure](#image-structure)
- [Image Enconding](#image-encoding)
    - [Split channels](#split-channels)
    - [Convert color image](#convert-from-color-image-to-another)
    - [Concatenate images](#concatenate-images-numpyndarray)
    - [Drawing on image](#drawing-on-image)
- [Image Processing](#image-processing)
    - [Threshold style](#threshold-style)
    - [Simple thresholding](#simple-thresholding)
    - [Adaptive method](#adaptive-method)
    - [Color filtering](#color-filtering)
- [Contour/Edge Detection](#contour--edge-detection)
- [Video](#video)
    -[Import video](#import-video)
    -[Read video by frame](#read-video-by-frame)
    - [Show video](#show-video)
- [OpenCV + ROS](#opencv--ros)
- [CMakeList](#cmakelist)
- [Ball Tracking Logic](#ball-tracking-logic)

# Image Input Output
## Reading image
Python:

```python
import cv2

image = cv2.imread(image_path, flags)
```

C++:

```c++
#include <opencv2/opencv.hpp>   // Need to be included in CMakeList later

using namespace cv;

Mat image;
image = imread(const String& image_path, int flags);
```

Flags:
- IMREAD_COLOR (default, neglect transparency)
- IMREAD_GRAYSCALE
- IMREAD_UNCHANGED

## Display Image
Python:

```python
cv2.namedWindow("WinName", flags)   # Optional
cv2.moveWindow("WinName", x, y)     # Optional
cv2.imshow("WinName", image)

cv2.waitKey(int millisecond)    # If 0, it will wait until a key is pressed
cv2.destroyAllWindows()
```

C++:
```c++
namedWindow("WinName", flags);
imshow("WinName", InputArray Mat);   // Mat is the source image

waitKey(int millisecond);    // If 0, it will wait until a key is pressed
destroyAllWindows();
```

Flags:
- WINDOW_AUTOSIZE (default, cannot resize window later through GUI)
- WINDOW_NORMAL
- WINDOW_FULLSCREEN

## Save image
Python:

```python
cv2.imwrite(destination, image)
```

C++:
```c++
imwrite(destination, image);
```

# Image Structure
Type of image:
```python
type(image)
>> <class 'numpy.ndarray'>
```

Shape of image:
```python
image.shape
>> rows, columns, channels
```

Size of image:
```python
image.size
>> rows * columns * channels
```

# Image Encoding
## Split channels
Python:

```python
blue, green, red = cv2.split(color_image)
```

C++:
```c++
split( const Mat& Input, vector<Mat>& Output );
```

## Convert from color image to another
Python:

```python
hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
```

C++:
```c++
cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);
cvtColor(bgr_image, gray_image, COLOR_BGR2GRAY);
```

## Concatenate images (numpy.ndarray)
Python:

```python
import numpy as np

np.concatenate((image_1, image_2,...), axis = 0)
```

> Axis = 0 will add image to below. Axis = 1 will add image to right.

## Drawing on image
Python:

```python
cv2.line(image, start_coor, end_coor, color, thickness)
cv2.rectangle(image, top_left_coor, bottom_right_coor, color, thickness)
cv2.circle(image, center_coor, int radius, color, thickness)
cv2.putText(image, "Text", bottom_left_loc, font, fontScale, color, thickness, lineType)
```

C++
```
Refer to Python
```

> Note that all locations are tuples and their elements need to be integer, and color is always in tuple(B,G,R)

# Image Processing
## Threshold style

- THRESH_BINARY (白->白, 黑->黑)
> Value = MAX , IF value > threshold <br>
> Value = &nbsp;0 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; , ELSE <br>

- THRESH_BINARY_INV (白->黑, 黑->白)
> Value = &nbsp;0 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; , IF value > threshold <br>
> Value = MAX , ELSE

- THRESH_TRUNC (去白留黑)
> Value = threshold&nbsp;&nbsp; , IF value > threshold <br>
> Value = unchanged , ELSE

- THRESH_TOZERO (不够白变黑)
> Value = unchanged , IF value > threshold <br>
> Value = 0 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; , ELSE

- THRESH_TOZERO_INV (太白变黑)
> Value = 0 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; , IF value > threshold <br>
> Value = unchanged, ELSE

## Simple thresholding
Simple thresholding takes all the individual pixel into calculation. Perform using only one threshold value.

Python:
```python
return_value, processed_image = cv2.threshold(image, 
                                              threshold_value, 
                                              max_value,
                                              threshold_style)
```

C++:
```c++
threshold(image, processed_image, threshold_value, max_value, threshold_style);
```

## Adaptive method
Adaptive method will take neighbouring pixels, calculate the weighted mean and use different threshold value based on the cluster.

Python:
```python
processed_image = cv2.adaptive_thresholding(image, 
                                  max_value, 
                                  adaptive_method, 
                                  threshold_style, 
                                  block_size,   # neighbour size
                                  constant)     # constant to minus away
```

C++:
```c++
adaptiveThreshold(image, processed_image, max_value, adaptive_method, threshold_style, block_size, constant);
```

Adaptive_method:
- ADAPTIVE_THRESH_MEAN_C
- ADAPTIVE_THRESH_GAUSSIAN_C

## Color filtering
Usually use HSV over BGR because it is more robust towards lighting change.

Python:
```python
processed__image = cv2.inRange(hsv_image, lower_bound, upper_bound)
```

C++:
```c++
inRange(hsv_image, Scalar lower_bound, Scalar upper_bound, processed_image);
```

> Boundaries are tuple of (hue,saturation,value) <br>
> In OpenCV, hue: 0~180 , saturation: 0~255 , value: 0~255

# Contour / Edge Detection
Python:

```python
contours, heirarchy = cv2.findContours(processed_image, mode, method)

cv2.drawContours(image, contours, contourIdx, color, (optional) thickness)
# Contours take input of a list
# Use contourIdx = -1 to draw all contours

area = cv2.contourArea(c)                   # Only one contour
perimeter = cv2.arcLength(c, True)          # Only one contour
((x,y), radius) = cv2.minEnclosingCircle(c) # Only one contour
```

C++:
```c++
vector<vector<Point>> contours;
vector<Vec4i> heirarchy;
findContours(processed_image, contours, heirarchy, mode, method);
// contours.size() can return number of contours

drawContours(image, contours, contourIdx, color, (optional) thickness);
// Use contourIdx = -1 to draw all contours

Point2f center;
float radius;
double area = contourArea(contour);             // One contour
double perimeter = arcLength(contour, True);    // One contour
minEnclosingCircle(contour, center, radius);    // One contour
```

# Video
## Import video
Python:
```python
video = cv2.VideoCapture("file_path")
video = cv2.VidepCapture(0) # Use webcam
```

C++:
```c++
VideoCapture video("video_path");
VideoCapture video(0);  // Use webcam
```

## Read video by frame
Python:
```python
while(True):
    return_val, frame = video.read()        # A frame is basically image
    if cv2.waitKey(1) & 0xFF == ord('q'):   # Use button q to exit
        break

video.release()
cv2.destroyAllWindows()
```

C++:
```c++
Mat frame;
while(true) {
    video >> frame;

    if((waitKey(30) >= 0)) break;
}
```

> waitKey() function is extremely important to ensure video plays properly.

## Show video
Python:
```python
cv2.imshow("WinName", frame)
```

C++:
```c++
imshow("WinName", frame)
```
# OpenCV + ROS
To use ROS to publish image to topic or subscribe image from topic, cv_bridge is needed and image_transport for c++.

Python:
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError   # Error message is optional

bridge = CvBridge()

# Subscribe
def imageCallback(ros_image):
    # To get image from ROS topic and convert into cv2 format
    cv2_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

subscriber = rospy.Subscriber("topic", Image, imageCallback)

# Publish
publisher = rospy.Publisher("topic", Image)
image_to_publish = bridge.cv2_to_imgmsg(src, "bgr8")
publisher.publish(image_to_publish)
```

C++:
```c++
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>

// Subscribe
void imageCallback(const sensor_msgs::ImageConstPtr& ros_image) {
    cv2_image = cv_bridge::toCvCopy(ros_image, "bgr8")->image;
}

ros::NodeHandle(nh);

image_transport::ImageTransport it(nh);
image_transport::Subscriber sub;
sub = it.subscribe("topic", queue_size, imageCallback);

// Publish
image_transport::Publisher pub;
pub = it.advertise("topic", queue_size);

sensor_msgs::ImagePtr image_to_publish;
image_to_publish = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();

pub.publish(image_to_publish);
```

# CMakeList
Make changes as below:
```txt
find_package (
    OpenCV          # Add this line
    cv_bridge       # Add this if working with ROS
    image_transport # Add this if working with ROS
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}  # Add this line
)

target_link_libraries(executable_name ${OpenCV_LIBRARIES})  # Add this line for all executable that uses OpenCV
```

# Ball Tracking Logic
1. Import video
2. Read video by frame
3. Convert image into hsv
4. Create a mask
5. Draw contour