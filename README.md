
# Robeff UDP Camera ROS2 Driver

The robeff_udp_cam_ros2_driver package provides a ROS2 node that interfaces with a UDP camera, receives image data via UDP packets, and publishes the images as ROS2 sensor_msgs/msg/Image messages. This package is designed to be a simple driver for integrating UDP-based cameras into a ROS2 ecosystem.


## Features

    Receives image data from a UDP camera.
    Converts the received UDP packets into OpenCV images.
    Publishes the images on a ROS2 topic as sensor_msgs/msg/Image.
    Optional image visualization mode using OpenCV.


## Requirements

- ROS2 Humble
- OpenCV
- cv_bridge
- rclcpp
- sensor_msgs


## Installation

Clone the repository into your ROS2 workspace:

```bash
  cd ~/ros2_ws/src
  git clone <repository-url>
```
Build the package:
```bash
  cd ~/ros2_ws
  colcon build --symlink-install --packages-select robeff_udp_cam_ros2_driver
```

## Usage/Examples
 
 Run the node with the default parameters:

```bash
ros2 run robeff_udp_cam_ros2_driver robeff_udp_cam_ros2_driver
```
or

```bash
ros2 launch robeff_udp_cam_ros2_driver robeff_udp_cam_ros2_driver.launch.py
```

## Parameters

- publisher_topic_image (string): The topic on which the image messages will be published. Default: "robeff/udp_cam/image_raw".

- frame_id (string): The frame ID for the image messages. Default: "robeff_udp_cam".

- udp_cam_ip (string): The IP address of the UDP camera. Default: "0.0.0.0".

- udp_cam_port (int): The port on which to listen for UDP packets. Default: 12345.

- image_viz_mode (bool): If true, the received images will be displayed using OpenCV. Default: false.


# Raspberry Pi Camera UDP Video Streamer

In the RaspberryScripts directory, there is a Python script that allows you to transmit data via UDP using the Raspberry Pi camera.

This Python script, located in the RaspberryScripts directory, enables video streaming from the Raspberry Pi camera over UDP. The script performs the following functions:

Camera Initialization: The script configures and starts the Raspberry Pi camera. If the camera fails to start, it will print an error message and retry after a short delay.

Video Streaming: The script continuously captures video frames from the camera, encodes each frame in JPEG format, and then transmits the encoded data in small chunks (1024 bytes) via UDP to a specified IP address and port.

Resource Management: Upon exiting, the script properly closes the socket and stops the camera to free up resources.

This script is useful for real-time video streaming from a Raspberry Pi to another device or application that can receive and process UDP packets.