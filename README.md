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

