# ROS Notes for Python

This note contains basic roscpp implementation.

# Content
- [ROS C++](#ros-c)
    - [Node](#node)
    - [Publisher](#publisher)
    - [Subscriber](#subscriber)
    - [Service (server)](#service-server)
    - [Service (client)](#service-client)
    - [ROS Param](#ros-param)
    - [CMakeList](#cmakelist)

# ROS C++
## Node
```c++
#include <ros/ros.h>

int main (int argc, char** argv) {
    ros::inite(argc, argv, "NodeName", (optional) ros::init_options::AnonymousName);
    ros::NodeHandle nh;
}
```

Anonymous argument enables multiple nodes running same code

## Publisher
```c++
#include <package_name/MsgType.h>

ros::Publisher pub;
pub = nh.advertise<package_name::MsgType>("topic", int queue_size);
ros::Rate rate(hz);

package_name::MsgType msg;
while(ros::ok()) {
    pub.publish(msg);
    ros::spinOnce();    // Optional for publisher
    rate.sleep();
}
```

## Subscriber
```c++
#include <package_name/MsgType.h>

void callback(package_name::MsgType msg) {
    // do something
}

ros::Subscriber sub;
sub = nh.subscribe("topic", int queue_size, callback);
ros::topic::waitForMessage<package_name::MsgType>("topic"); // Optional

ros::spin();
```

## Service (server)
```c++
#include <package_name/Srv.h>

bool handler(package_name::Srv::Request &req, 
            package_name::Srv::Response &res) 
{
    // do something
    // res.area = req.length * req.width;
    return true;
}

ros::ServiceServer server;
server = nh.advertiseService("service_name", handler);

ros::spin();
```

Compared to Python, C++ has a handler that returns boolean expression. Response need to be updated in the handler function explicitly.

## Service (client)
```c++
#include <package_name/Srv.h>

ros::ServiceClient client;
client = nh.serviceClient<package_name::Srv>("service_name");

package_name::Srv srv_msg;
srv_msg.request.a = a;

if(client.call(srv_msg)) {  // Client will send request to server, returned with boolean expression
    // do something
    // response = srv_msg.response.b;   // Response can be obtained like this
}
```

## ROS Param
```c++
#include <package_name/MsgType.h>

nh.set_param("param_name", param_value);
package_name::MsgType var;
nh.get_param("param_name", var);
```

## CMakeList
```txt
add_executable(executable_name src/.../FileName.cpp)
target_link_libraries(executable_name ${catkin_LIBRARIES})
```