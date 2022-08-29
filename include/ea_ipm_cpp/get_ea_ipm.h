//
// Created by pjy on 22. 8. 9.
//
#pragma once

#include <memory>
#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>


class EA_IPM : public rclcpp::Node // node class 상속
{
    public:
        EA_IPM();
        ~EA_IPM() {};
  

    private:
        // camera intrinsic
        float fx = 1318.93;
        float fy = 1320.25;
        float cx = 1337.66;
        float cy = 1004.15;

        // camera extrinsic
        float h = 0.561;
        float tilt = 0.0;

        float theta = -tilt;
        
        // euler angle
        double imu_roll, imu_pitch, imu_yaw;
        double odom_roll, odom_pitch, odom_yaw;

        sensor_msgs::msg::PointCloud2 IPM_points;
        std::string parent_frame;
    
        cv::Mat get_roi_img(cv::Mat img);
        sensor_msgs::msg::PointCloud2 convert_PC2(std::vector<std::vector<float>> point_fields);
        void imu_quat2euler(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
        void odom_quat2euler(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
        void get_ea_ipm(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg);

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ipm_publisher;
};
