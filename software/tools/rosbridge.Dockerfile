# rosbridge for the Qutee GUIs (joint reader, homeokinesis): ROS 2 Humble with rosbridge_server (JointState and parameters are
# built-in types). The GUI (roslibpy) connects to its WebSocket on port 9090; no ROS install needed on the PC.
#   docker build -t qutee_rosbridge_basic -f rosbridge.Dockerfile .   (from software/tools)
FROM ros:humble
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*
CMD ["bash", "-c", ". /opt/ros/humble/setup.sh && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 call_services_in_new_thread:=true"]
