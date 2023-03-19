# ROS Notes for Python

This note contains basic ROS usage such as node, publisher, subscriber, customization of msg & srv, create launch and etc. Also the python implementation using rospy.

# Content
- [Catkin Workspace](#catkin-workspace)
    - [Setup](#setup-a-catkin-workspace)
- [ROS Basics](#ros-basics)
    - [Import ROS](#import-ros)
    - [Node](#node)
    - [Publisher](#publisher)
    - [Subscriber](#subscriber)
    - [Service (server)](#service-server)
    - [Service (client)](#service-client)
    - [Customizing msg / srv](#customizing-msg--srv)
        - [Using customized msg & srv](#using-customized-msg--srv)
    - [ROS Param](#ros-param)
    - [Launch file](#launch-file)
    - [ROS commands](#ros-commands)

# Catkin workspace
## Setup a catkin workspace
Create a catkin_ws folder, go inside to create src folder and run catkin_make. Scripts will be stored in src folder.

```bash
> mkdir catkin_ws && cd catkin_ws
> mkdir src && catkin_make
```

Add setup.bash into .bashrc

```bash
> echo "source /opt/ros/[distro]/setup.bash" >> ~/.bashrc
> echo "source /[directory]/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Go to src folder and create a new package.

```bash
> cd src/
> catkin_create_pkg package_name dependencies(eg: roscpp rospy std_msgs)
```

# ROS Basics
## Import ROS
```python
#!/usr/bin/env python

import rospy
```

## Node
```python
rospy.init_node( "node_name", (optional) bool:Anonymous )
```

Anonymous argument enables multiple nodes running same code

## Publisher
```python
from package_name.msg import MsgType

pub = rospy.Publisher("topic", MsgType, queue_size)
rospy.Rate rate(hz)

msg = MsgType()
while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
```

## Subscriber
```python
from package_name.msg import MsgType

def callback(msg):
    some action

sub = rospy.Subscriber("topic", MsgType, callback)
rospy.wait_for_message("topic", MsgType)  # Optional, make sure initialization complete

rospy.spin()
```

## Service (server)
```python
from package_name.srv import SrvType

def handler(request):
    # Do something
    # b = request.a
    return result

server = rospy.Service("service_name", SrvType, handler)
```

Result is returned in the handler function

## Service (client)
```python
from package_name.srv import SrvType

rospy.wait_for_service("service_name")
client = rospy.ServiceProxy("service_name", SrvType)
result = client(var1, var2,...)
```

## Customizing msg / srv
1. It is a good practice to create an individual package, for msg customization and srv customization.

```bash
> cd ~/.../catkin_ws/src/
> catkin_create_pkg msg_srv_pack dependencies
> cd msg_srv_pack && rm -r include src
```

2. Create msg folder for msg, srv folder for srv

```bash
> cd ~/.../catkin_ws/src/msg_srv_pack/
> mkdir msg && cd msg
> touch MsgName.msg

> cd ~/.../catkin_ws/src/msg_srv_pack/
> mkdir srv && cd srv
> touch SrvName.srv
```

3. Can start create the customized msg or srv
```
> RobotState.msg
string name
float64 battery

> CalculateArea.srv
float64 length
float64 width
---
float64 area
```

4. Open the package.xml file and add these two lines

```xml
<build_depend> message_generation </build_depend>
<exec_depend> message_runtime </exec_depend>
```

5. Make these changes in CMakeList
```txt
find_package(
    message_generation  # Add this line
)

add_message_files(      # Uncomment this block
    FILES
    MsgName.msg         # Add customized msg file
)

add_service_files(      # Uncomment this block
    FILES
    SrvName.srv         # Add customized srv file
)

generate_messages(      # Uncomment this block
    DEPENDENCIES
    [dependencies]      # Add relevant dependencies
)

catkin_package(
    CATKIN_DEPENDS  message_runtime # Uncomment and add this line
)
```

6. Go back to catkin_ws & catkin_make
```bash
> cd ~/.../catkin_ws/
> catkin_make
```

7. Header files will be generated in catkin_ws/devel folder
```
> ~/.../catkin_ws/devel/include/[msg_srv_pack]/[MsgSrvName].h
```

### Using customized msg & srv
1. Open package.xml and add line as following
```xml
<depend> msg_srv_pack_name </depend>
```

2. In CMakeList, make these changes
```txt
find_package(
    msg_srv_pack_name   # Add this line
)
```

## ROS Param
```python
rospy.set_param("param_name", value)
param_value = rospy.get_param("param_name")
```

## Launch file
1. Create a folder launch in the package folder
```bash
> cd ~/.../catkin_ws/src/package_name
> mkdir launch
```

2. Create a launch file in launch folder
```bash
> cd ~/.../catkin_ws/src/package_name/launch
> touch Name.launch
```

3. Launch file can be created as follow
```launch
<launch>
    <node name="node_name" pkg="pkg_name" type="executable_name">
        <param name="param_name" value="param_value" (optional) type="type" />
        <remap from="old_value" to="new_value" />
    </node>
</launch>
```

The executable name must align to the name defined in CMakeList, add_executable(). For python file, just put as the file_name.py

## ROS commands
```ros
> roscore
> rostopic list / echo / info
> rosnode list / echo / info
> rosservice list / call / info
> rosmsg show
> rossrv show
```