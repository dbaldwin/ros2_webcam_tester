# Overview
This was a quick project I threw together to test getting a camera from my Mac host into a ROS2 container. It's also an experiment related to testing image_raw vs image_raw/compressed frames and finding the ideal solution.

# Update the system
sudo apt update

sudo apt install ros-humble-compressed-image-transport

# Start a ROS2 container
docker run -p 6080:80 -p 5005:5005/udp -v ${PWD}:/home/ubuntu/ros2_webcam_ws/src --name=ros2-webcam-tester --security-opt seccomp=unconfined --shm-size=512m ghcr.io/tiryoh/ros2-desktop-vnc:humble

# Fix numpy so we can run the node
pip install "numpy<2" --force-reinstall

# Build the node
cd ~/ros2_webcam_ws

colcon build

source install/setup.bash

# Start broadcasting video from Mac host
ffmpeg -f avfoundation -framerate 30 -video_size 640x480 -i "0" -f mpegts udp://127.0.0.1:5005

# Run the node
ros2 run ros2_webcam_tester camera_node

# View the output
ros2 run rqt_image_view rqt_image_view


# Troubleshooting
If you get a "failed to capture image" error you may need to restart docker. I believe this may be due to a port conflict as we're passing camera frames into the container from the host over UDP.

