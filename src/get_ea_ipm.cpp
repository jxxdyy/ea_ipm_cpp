//
// Created by pjy on 22. 8. 9.
//

#include "ea_ipm_cpp/get_ea_ipm.h"

using std::placeholders::_1;


EA_IPM::EA_IPM(): Node("EA_ipm")
{
    image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/cam_f/image/compressed", 10, std::bind(&EA_IPM::get_ea_ipm, this, _1));
    // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu", 10, std::bind(&EA_IPM::imu_quat2euler, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odomgyro", 10, std::bind(&EA_IPM::odom_quat2euler, this, _1));
    ipm_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/IPM_points", 10);
}


void EA_IPM::imu_quat2euler(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{   
    tf2::Quaternion quat(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w
    );
    
    tf2::Matrix3x3 m(quat);
    m.getRPY(imu_roll, imu_pitch, imu_yaw);

    std::cout << imu_pitch << std::endl;
  
}


void EA_IPM::odom_quat2euler(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{   
    tf2::Quaternion quat(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    
    tf2::Matrix3x3 m(quat);
    m.getRPY(odom_roll, odom_pitch, odom_yaw);

    theta_p = -odom_pitch;
    theta_r = -odom_roll;   
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

    const cv::Point* ppt[1] = { points };
    int npt[] = { 6 }; // npt[0] = 6

    // fillfoly(입력 이미지, 포인트 배열, 점의 개수, 테두리 수(1), 색, 라인 타입)
    cv::fillPoly(roi_mask, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
    bitwise_and(img, roi_mask, roi_img);

    return roi_img;
}


cv::Mat EA_IPM::get_IPM_img(std::vector<std::vector<float>> point_fields)
{ 
    cv::Size IPM_size(512, 512);
    cv::Mat IPM_img(IPM_size, CV_8UC3); //CV_8UC3 : RGB scale

    float vir_cam_x = 2.0;
    float vir_cam_y = 0.0;
    float vir_cam_z = 1.0;
    float vir_cam_pan = -90.0;
    float vir_cam_tilt = -90.0;



    // polygon을 생성할 좌표


    return IPM_img;
}


sensor_msgs::msg::PointCloud2 EA_IPM::convert_PC2(std::vector<std::vector<float>> point_fields)
{
    sensor_msgs::msg::PointCloud2 ea_ipm_pc2;
    std_msgs::msg::Header header;
    sensor_msgs::msg::PointField fields;
    std::vector<sensor_msgs::msg::PointField> v_fields;
    unsigned int ros_type = sensor_msgs::msg::PointField::FLOAT32;
    unsigned int itemsize = sizeof(ros_type);
    
    std::string name[6] = {"x", "y", "z", "b", "g", "r"};
    int name_size = sizeof(name)/sizeof(name[0]);
    unsigned int POINT_STEP = itemsize * name_size;

    for (int i=0; i<name_size; i++){
        fields.name = name[i];
        fields.offset = i * itemsize;
        fields.datatype = ros_type;
        fields.count = 1;
        v_fields.push_back(fields);
    }

    parent_frame = "eaipm";
    header.frame_id = parent_frame;

    ea_ipm_pc2.header = header;
    ea_ipm_pc2.height = 1;
    ea_ipm_pc2.width = point_fields.size();
    ea_ipm_pc2.is_dense = false;
    ea_ipm_pc2.is_bigendian = false;
    ea_ipm_pc2.fields = v_fields;
    ea_ipm_pc2.point_step = POINT_STEP;
    ea_ipm_pc2.row_step = (itemsize * name_size * point_fields.size());
    ea_ipm_pc2.data.resize(point_fields.size() * POINT_STEP, 0x00);

    uint8_t *ptr = ea_ipm_pc2.data.data();

    // reinterprest : 임의의 포인터 타입끼리 변환을 허용하는 cast 연산자
    for (size_t i = 0; i < point_fields.size(); i++)
    {
        *(reinterpret_cast<float*>(ptr + 0)) = point_fields[i][0];
        *(reinterpret_cast<float*>(ptr + 4)) = point_fields[i][1];
        *(reinterpret_cast<float*>(ptr + 8)) = point_fields[i][2];
        *(reinterpret_cast<float*>(ptr + 12)) = point_fields[i][3];
        *(reinterpret_cast<float*>(ptr + 16)) = point_fields[i][4];
        *(reinterpret_cast<float*>(ptr + 20)) = point_fields[i][5];
        ptr += POINT_STEP;
    }

    return ea_ipm_pc2;
}


Eigen::MatrixXf EA_IPM::adaptive_roll(float r, float c)
{
    Eigen::Matrix2f rot;
    Eigen::MatrixXf rc(1, 2), rot_rc(1, 2);

    rot << cos(theta_r), -sin(theta_r), sin(theta_r), cos(theta_r);
    rc << r, c;

    rot_rc = rc*rot;

    return rot_rc; 
}


void EA_IPM::get_ea_ipm(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
{
    //std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    cv_bridge::CvImagePtr front_img_ptr;
    cv::Mat front_img, roi_img;

    front_img_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    front_img = front_img_ptr->image;
    
    int img_col = front_img_ptr->image.cols;  // 2592
    int img_row = front_img_ptr->image.rows;  // 1944

    // ground를 보기 위한 ROI설정
    roi_img = get_roi_img(front_img);

    cv::namedWindow("front_image", 0);
    cv::resizeWindow("front_image", img_col/2, img_row/2);
    cv::imshow("front_image", roi_img);
    cv::waitKey(1);

    
    // =================================== EA-IPM 적용 부분 ===================================
    float theta_v, X, Y, Z;
    std::vector<std::vector<float>> point_fields;
    int row_increase = 2;
    int col_increase = 4;

    // 클래스의 멤버 변수를 사용하고자 할 때 this를 capture
    auto v2r = [this](int v_) {return cy + 0.5 - v_;};
    auto u2c = [this](int u_) {return u_ - cx + 0.5;};
    

    for (int v = img_row*3/5; v < img_row; v = v + row_increase){
        // 포인터를 이용하여 pixel 접근
        uchar* ptr_row = roi_img.ptr<uchar>(v);
        for (int u = 0; u < img_col; u = u + col_increase){
            float B = ptr_row[u * 3 + 0]; 
            float G = ptr_row[u * 3 + 1]; 
            float R = ptr_row[u * 3 + 2];
            //std::cout << b << " " << g << " " << r << std::endl;

            float r = v2r(v);
            float c = u2c(u);
            // adaptive roll
            // r = adaptive_roll(r, c)(0, 0);
            // c = adaptive_roll(r, c)(0, 1);

            theta_v = -atan(r/fx);

            X = h * (1 / tan((theta) + theta_v));
            Y = -(cos(theta_v) / cos((theta) + theta_v)) * (X * c / fx);
            Z = 0;

            point_fields.push_back({X, Y, Z, B, G, R});
        }
    }
    //std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    //std::chrono::nanoseconds nano = end - start;
    //std::cout << "c++ 코드 실행 시간(나노초 ns) : " << nano.count() << std::endl;

    IPM_points = convert_PC2(point_fields);
    ipm_publisher->publish(IPM_points); 


    std::cout << "points size :" << point_fields.size() << std::endl;
    std::cout << "=============================================" << std::endl;
 
}

