//
// Created by pjy on 22. 8. 9.
//

#include "ea_ipm/get_ea_ipm.h"

using std::placeholders::_1;


EA_IPM::EA_IPM(): Node("EA_ipm")
{
    image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/cam_f/image/compressed", 10, std::bind(&EA_IPM::topic_callback, this, _1));
}

cv::Mat EA_IPM::get_roi_img(cv::Mat img)
{ 
    cv::Mat roi_mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC3); // CV_8UC3 : RGB scale
    cv::Mat roi_img;

    // polygon을 생성할 좌표
    cv::Point points[6];
    points[0] = cv::Point(0, img.rows);
    points[1] = cv::Point(0, img.rows*9/10);
    points[2] = cv::Point(img.cols/4, img.rows*3/5);
    points[3] = cv::Point(img.cols*3/4, img.rows*3/5);
    points[4] = cv::Point(img.cols, img.rows*9/10);
    points[5] = cv::Point(img.cols, img.rows);


    for (int i=0; i<6; i++){
        std::cout << points[i] << std::endl;
    }

    const cv::Point* ppt[1] = { points };
    int npt[] = { 6 }; // npt[0] = 6

    // fillfoly(입력 이미지, 포인트 배열, 점의 개수, 테두리 수(1), 색, 라인 타입)
    cv::fillPoly(roi_mask, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
    bitwise_and(img, roi_mask, roi_img);

    return roi_img;
} 


void EA_IPM::topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv_bridge::CvImagePtr front_img_ptr;
    int image_col, image_row;
    cv::Mat mask;

    front_img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    image_col = front_img_ptr->image.cols;
    image_row = front_img_ptr->image.rows;


    mask = get_roi_img(front_img_ptr->image);

    std::cout << "mask :" << mask.size() << std::endl;
    std::cout << "====================================" << std::endl;


    cv::namedWindow("front_image", 0);
    cv::resizeWindow("front_image", front_img_ptr->image.cols/2, front_img_ptr->image.rows/2);
    cv::imshow("front_image", mask);
    cv::waitKey(1);
}