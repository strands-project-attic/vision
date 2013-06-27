#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    std::cout << "Hey, listen!" << std::endl;

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        cv::imshow("foo", cv_ptr->image);
        cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "Oh hai there!" << std::endl;

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
    ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgcb);

    cv::namedWindow("foo");
    ros::spin();
    cv::destroyWindow("foo");

    std::cout << "byebye my friend" << std::endl;

    return 0;
}

