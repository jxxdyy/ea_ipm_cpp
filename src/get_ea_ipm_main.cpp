#include "ea_ipm/get_ea_ipm.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EA_IPM>());
    rclcpp::shutdown();

    return 0;
}