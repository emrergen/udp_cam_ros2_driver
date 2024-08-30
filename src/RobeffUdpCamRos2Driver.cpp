#include "robeff_udp_cam_ros2_driver/RobeffUdpCamRos2Driver.hpp"

namespace robeff
{
    namespace udp_cam_ros2_driver
    {

        UdpCamDriver::UdpCamDriver(const std::string &t_node_name) : Node(t_node_name), m_node(this)
        {
            RCLCPP_INFO(m_node->get_logger(), "[%s] UDP Camera Driver Started", __APP_NAME__);
            if (!readParameters())
            {
                RCLCPP_ERROR(m_node->get_logger(), "[%s] Failed to read parameters", __APP_NAME__);
                return;
            }

            m_publisher_topic_image = m_parameter_publisher_topic_image.get_value<std::string>();
            m_frame_id = m_parameter_frame_id.get_value<std::string>();
            m_udp_cam_ip = m_parameter_udp_cam_ip.get_value<std::string>();
            m_udp_cam_port = m_parameter_udp_cam_port.get_value<int>();
            m_image_viz_mode = m_parameter_image_viz_mode.get_value<bool>();

            m_publisher_image = m_node->create_publisher<sensor_msgs::msg::Image>(m_publisher_topic_image, 10);

            RCLCPP_INFO(m_node->get_logger(), "[%s] UDP Camera IP: %s", __APP_NAME__, m_udp_cam_ip.c_str());
            RCLCPP_INFO(m_node->get_logger(), "[%s] UDP Camera Port: %d", __APP_NAME__, m_udp_cam_port);
            RCLCPP_INFO(m_node->get_logger(), "[%s] Publisher Topic Image: %s", __APP_NAME__, m_publisher_topic_image.c_str());
            RCLCPP_INFO(m_node->get_logger(), "[%s] Frame ID: %s", __APP_NAME__, m_frame_id.c_str());
            RCLCPP_INFO(m_node->get_logger(), "[%s] Image Viz Mode: %s", __APP_NAME__, m_image_viz_mode ? "true" : "false");

            udpCamCallback();
        }

        UdpCamDriver::~UdpCamDriver()
        {
        }

        bool UdpCamDriver::readParameters()
        {

            this->declare_parameter<std::string>("publisher_topic_image", "a");
            this->declare_parameter<std::string>("frame_id", "b");
            this->declare_parameter<std::string>("udp_cam_ip", "c");
            this->declare_parameter<int>("udp_cam_port", 0);
            this->declare_parameter<bool>("image_viz_mode", false);

            this->get_parameter("publisher_topic_image", m_parameter_publisher_topic_image);
            this->get_parameter("frame_id", m_parameter_frame_id);
            this->get_parameter("udp_cam_ip", m_parameter_udp_cam_ip);
            this->get_parameter("udp_cam_port", m_parameter_udp_cam_port);
            this->get_parameter("image_viz_mode", m_parameter_image_viz_mode);

            m_publisher_topic_image = m_parameter_publisher_topic_image.as_string();
            m_frame_id = m_parameter_frame_id.as_string();
            m_udp_cam_ip = m_parameter_udp_cam_ip.as_string();
            m_udp_cam_port = m_parameter_udp_cam_port.as_int();
            m_image_viz_mode = m_parameter_image_viz_mode.as_bool();

            if (m_publisher_topic_image.empty() || m_frame_id.empty() || m_udp_cam_ip.empty() || m_udp_cam_port == 0)
            {
                std::cout << "Publisher Topic Image: " << m_publisher_topic_image << std::endl;
                std::cout << "Frame ID: " << m_frame_id << std::endl;
                std::cout << "UDP Camera IP: " << m_udp_cam_ip << std::endl;
                std::cout << "UDP Camera Port: " << m_udp_cam_port << std::endl;
                std::cout << "Image Viz Mode: " << m_image_viz_mode << std::endl;
                RCLCPP_ERROR(get_logger(), "[%s] Parameters not set", __APP_NAME__);
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "Publisher Topic Image: %s", m_publisher_topic_image.c_str());
            RCLCPP_INFO(this->get_logger(), "Frame ID: %s", m_frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "UDP Camera IP: %s", m_udp_cam_ip.c_str());
            RCLCPP_INFO(this->get_logger(), "UDP Camera Port: %d", m_udp_cam_port);
            RCLCPP_INFO(this->get_logger(), "Image Viz Mode: %s", m_image_viz_mode ? "true" : "false");

            return true;
        }

        void UdpCamDriver::udpCamCallback()
        {
            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(m_udp_cam_port);
            addr.sin_addr.s_addr = inet_addr(m_udp_cam_ip.c_str());

            if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(get_logger(), "[%s] Failed to bind socket", __APP_NAME__);
                return;
            }

            RCLCPP_INFO(get_logger(), "[%s] UDP Camera Callback Started", __APP_NAME__);

            if (m_image_viz_mode)
            {
                cv::namedWindow("Receiver", cv::WINDOW_NORMAL);
            }

            std::vector<uint8_t> data;
            data.reserve(65535); // Maximum UDP packet size

            while (rclcpp::ok())
            {
                uint8_t buffer[1024];
                int recv_len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
                if (recv_len > 0)
                {
                    data.insert(data.end(), buffer, buffer + recv_len);

                    if (recv_len < 1024)
                    {
                        cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);
                        if (!frame.empty())
                        {
                            if (m_image_viz_mode)
                            {
                                auto now = std::chrono::system_clock::now();
                                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                                std::string time_str = "Receiver: " + std::string(std::ctime(&now_time));
                                cv::putText(frame, time_str, cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                            }

                            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                            img_msg->header.frame_id = m_frame_id;
                            m_publisher_image->publish(*img_msg);
                        }

                        data.clear();
                    }
                }
            }

            close(sock);
            cv::destroyAllWindows();
        }

    } // namespace udp_cam_ros2_driver

} // namespace robeff
