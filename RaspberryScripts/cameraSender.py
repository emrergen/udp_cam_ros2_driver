#!/usr/bin/env python3
# Raspberry Pi Camera UDP Video Streamer
# Author: Said Emre Ergen 2024

import cv2
import socket
from picamera2 import Picamera2
from time import sleep

def udp_camera_publisher(receiver_ip, receiver_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    is_camera_started = 0
    while not is_camera_started:
        try:
            picam2 = Picamera2()
            camera_config = picam2.create_video_configuration(main={"size": (1280, 720), "format": "RGB888"})
            camera_config['controls']["FrameDurationLimits"] = (25000, 25000) # 40 FPS 
            picam2.configure(camera_config)
            picam2.start()
            is_camera_started = 1
        except Exception as e:
            is_camera_started = 0
            picam2.stop()
            picam2.close()
            print(f"Camera not started: {e}")
            sleep(1)
    
    try:
        while True:
            frame = picam2.capture_array() 
            # frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate the frame if needed
            encoded, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])  
            image_data = buffer.tobytes()
            for i in range(0, len(image_data), 1024):
                chunk = image_data[i:i+1024]
                sock.sendto(chunk, (receiver_ip, receiver_port))
    finally:
        sock.close()
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    receiver_ip = "10.42.0.1"
    receiver_port = 12345
    
    udp_camera_publisher(receiver_ip, receiver_port)
