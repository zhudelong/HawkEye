#ifndef _SEARCHERS_H_
#define _SEARCHERS_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>



class searchers
{
public:
    searchers() {}

private:
    ros::Subscriber tag_local_position_subscriber;
    ros::Subscriber tag_global_position_subscriber;
    ros::Subscriber x3_info_subscriber;
    image_transport::Subscriber x3_image_subscriber;

public:
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);



};



#endif

