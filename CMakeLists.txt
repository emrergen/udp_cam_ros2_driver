cmake_minimum_required(VERSION 3.8)
project(robeff_udp_cam_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies "sensor_msgs" "rclcpp" "cv_bridge" "OpenCV")

include_directories(include)

add_library(robeff_udp_cam_ros2_driver_core
        src/RobeffUdpCamRos2Driver.cpp
        )
target_compile_features(robeff_udp_cam_ros2_driver_core PUBLIC cxx_std_17)

add_executable(robeff_udp_cam_ros2_driver
        src/robeff_udp_cam_ros2_driver_node.cpp
        src/RobeffUdpCamRos2Driver.cpp
        )
target_compile_features(robeff_udp_cam_ros2_driver PUBLIC cxx_std_17)

ament_target_dependencies(robeff_udp_cam_ros2_driver_core  ${dependencies})

ament_target_dependencies(robeff_udp_cam_ros2_driver ${dependencies})

target_link_libraries(robeff_udp_cam_ros2_driver_core
    ${OpenCV_LIBRARIES}
)

target_link_libraries(robeff_udp_cam_ros2_driver
    robeff_udp_cam_ros2_driver_core
    ${OpenCV_LIBRARIES}
)

install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
robeff_udp_cam_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
