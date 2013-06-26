kinect
======

Stuff to get the "kinect" data published in ros, and maybe do stuff with it.

Installation
============

What previously was known as `openni_kinect` is now split into two packages: [openni-camera](http://www.ros.org/wiki/openni_camera) and [openni-launch](http://www.ros.org/wiki/openni_launch).
Install them:

```bash
$ sudo apt-get install ros-groovy-openni-camera ros-groovy-openni-launch
```

Then, you can run the process which will publish the camera's data. (Of course, `roscore` should be running.)

```bash
$ roslaunch openni_launch openni.launch
```

Viewing the topics
==================

Check which topics are published by running `rostopic list` and, more interestingly, look at the streams using

```bash
$ rosrun image_view image_view image:=/camera/rgb/image_color
$ rosrun image_view image_view image:=/camera/depth/image
```

Troubleshooting
===============

Failed to set USB interface
---------------------------

This errro may occur when executing the `roslaunch openni...` command.

The ASUS Xtion has [firmware problems with USB 3.0 interfaces](http://reconstructme.net/2012/10/13/asus-xtion-usb-3-0-hotfix/).
Befor trying to patch the firmware (on Windows), just plug it into some other USB interface.
Modern laptops have some USB 3.0 and some USB 2.0 interfaces, so you might be lucky to hit a 2.0 one which just works.

Another possible solution, reported to work for some, is to load the following kernel modules:

```bash
$ sudo modprobe -r gspca_kinect
$ sudo modprobe -r gspca_main
```


Creating a C++ node listening to the Kinect
===========================================
Let's get going!

Creating a new project in ROS
-----------------------------

### Disclaimer
I don't know all the terminologies yet, I may use wrong words for things.

### Creating a package
First, creating a package:

```bash
$ cd ~/strands/sim/src
$ catkin_create_pkg test_listener rospy std_msgs
$ cd ..
$ catkin_make
```

There can be many "nodes" (apps/mains/...) in one package.
A node may be written in C++ or Python.

### Creating a C++ node
Write your code. Minimal code, in a file we'll call `test_ir` just for giggles:

```cpp
#include <iostream>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Foo_bar_baz_quux");

    std::cout << "Oh hai there!" << std::endl;

    return 0;
}
```

Add this thingy to the `CMakeLists.txt` file of your package, so that `catkin_make` knows what to do.
The previous step involving `catkin_create_pkg` did create a template `CMakeLists.txt` file
which you can have a look at and uncomment the things you want. For our minimal project that
results in:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(vision_node test_ir.cpp)

target_link_libraries(vision_node
  ${catkin_LIBRARIES}
)

install(TARGETS vision_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Then, you need to compile your package by running `catkin_make`. But be wary, you'll get a cryptic error
message if you run it in the wrong folder. Run it from `~/strands/sim` in this case.

If it all succeeds, you can run the `vision_node` executable in the `vision` package by then typing:

```bash
$ rosrun vision vision_node
```

It should greet you warmly.
