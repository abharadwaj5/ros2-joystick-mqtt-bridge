# ROS 2 side of the remote robot assignment.
#official ROS 2 Jazzy,
FROM ros:jazzy

# libmosquitto-dev   - MQTT client library used by the bridge
# nlohmann-json3-dev - header-only JSON parser
# mosquitto-clients  - mosquitto_pub/sub, useful for testing from inside the container
RUN apt-get update && apt-get install -y --no-install-recommends \
        libmosquitto-dev \
        nlohmann-json3-dev \
        mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros2_ws/src ./src

RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

 #this sources the workspace overlay before starting the node.
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 run remote_robot_bridge mqtt_bridge"]