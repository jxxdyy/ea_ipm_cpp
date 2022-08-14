//
// Created by pjy on 22. 8. 9.
//
#pragma once

#include <memory>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>


class EA_IPM : public rclcpp::Node // node class 상속
{
    public:
        EA_IPM();
        ~EA_IPM() {};
  
    private:
        cv::Mat get_roi_img(cv::Mat img);
        void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
};
