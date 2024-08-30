#include "robeff_udp_cam_ros2_driver/RobeffUdpCamRos2Driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"UdpCamRos2DriverNode"};
    std::shared_ptr<robeff::udp_cam_ros2_driver::UdpCamDriver> ros2_node{nullptr};
    ros2_node = std::make_shared<robeff::udp_cam_ros2_driver::UdpCamDriver>(t_node_name);
    rclcpp::spin(ros2_node);
    rclcpp::shutdown();

    return 0;
}