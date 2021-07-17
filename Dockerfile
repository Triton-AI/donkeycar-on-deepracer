FROM osrf/ros:foxy-desktop

# Essentials
RUN apt update && apt-key adv --refresh-keys && apt upgrade -y && rosdep update

RUN apt install -y nano rsync git tio gdb

# ROS 2
WORKDIR /root/ros2_ws/
SHELL [ "/bin/bash", "-c" ]
RUN mkdir src/

# Grab the packages
RUN cd src && \
    git clone https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg && \
    git clone https://github.com/aws-deepracer/aws-deepracer-camera-pkg && \
    git clone https://github.com/aws-deepracer/aws-deepracer-servo-pkg

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -i -y

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

RUN apt install -y python3 python3-pip
RUN python3 -m pip install opencv-python

# Done
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc