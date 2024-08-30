// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

// C++ Libraries
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <chrono>

#define __APP_NAME__ "robeff_udp_cam_ros2_driver"

namespace robeff
{
    namespace udp_cam_ros2_driver
    {

        /*!
         * Main class for the node to handle the ROS interfacing.
         */
        class UdpCamDriver : public rclcpp::Node
        {
            // Inherit Node Constructors
            using rclcpp::Node::Node;

        public:
            /*!
             * Constructor.
             */
            UdpCamDriver(const std::string &t_node_name);

            /*!
             * Destructor.
             */
            ~UdpCamDriver();

        private:
            /*!
             * Reads the only std library variables
             * Reads and verifies the ROS parameters.
             * @return true if successful.
             */
            inline bool readParameters();

            /*!
             * UDP Camera Callback
             * Receives the UDP packets and publishes the image.
             */
            void udpCamCallback();
            
            //! rclcpp node variable
            rclcpp::Node::SharedPtr m_node;

            //! rclcpp publisher variable
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher_image;

            //! UDP Camera Publisher Topic
            std::string m_publisher_topic_image;

            //! Frame ID
            std::string m_frame_id;

            //! UDP Camera IP
            std::string m_udp_cam_ip;

            //! Image Visualization Mode
            bool m_image_viz_mode;

            //! UDP Camera Portq
            int m_udp_cam_port;

            //! ROS2 Parameter for Publisher Topic Image
            rclcpp::Parameter m_parameter_publisher_topic_image;

            //! ROS2 Parameter for Frame ID
            rclcpp::Parameter m_parameter_frame_id;

            //! ROS2 Parameter for UDP Camera IP
            rclcpp::Parameter m_parameter_udp_cam_ip;

            //! ROS2 Parameter for UDP Camera Port
            rclcpp::Parameter m_parameter_udp_cam_port;

            //! ROS2 Parameter for Image Visualization Mode
            rclcpp::Parameter m_parameter_image_viz_mode;
        };

    } // namespace udp_cam_ros2_driver

} // namespace robeff`
