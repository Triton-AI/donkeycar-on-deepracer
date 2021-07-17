FROM ros:foxy

# Essentials
RUN apt update && apt-key adv --refresh-keys && apt upgrade -y && rosdep update

RUN apt install -y nano rsync git tio gdb python3 python3-pip

RUN python3 -m pip install opencv-python

# ROS 2
WORKDIR /root/ros2_ws/
SHELL [ "/bin/bash", "-c" ]
RUN mkdir src/

# Grab the packages
RUN cd src && \
    git clone https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg && \
    git clone https://github.com/aws-deepracer/aws-deepracer-camera-pkg && \
    git clone https://github.com/aws-deepracer/aws-deepracer-servo-pkg && \
    git clone -b dev https://github.com/Triton-AI/donkeycar-on-deepracer 

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -i -y
RUN apt install libjsoncpp-dev

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Done
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "ros2", "launch", "donkey_on_deepracer", "donkey_on_deepracer.launch.py" ]