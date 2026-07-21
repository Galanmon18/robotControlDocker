FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true

RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-catkin-tools \
    iputils-ping \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-eigen-conversions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git \
        src/Universal_Robots_ROS_Driver \
    && git clone -b calibration_devel \
        https://github.com/fmauch/universal_robot.git \
        src/fmauch_universal_robot

COPY uma_ur_launch/ src/uma_ur_launch/

COPY custom_ur_configs/*.yaml src/Universal_Robots_ROS_Driver/ur_robot_driver/config/

RUN apt-get update \
    && rosdep update --rosdistro noetic \
    && bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -y" \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /catkin_ws/user_ws/devel/setup.bash ]; then source /catkin_ws/user_ws/devel/setup.bash; fi" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
