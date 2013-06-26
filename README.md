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

