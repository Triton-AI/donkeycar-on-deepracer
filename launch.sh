source /opt/ros/foxy/setup.bash
source /opt/aws/deepracer/lib/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
source ~/projects/ros2_ws/install/setup.bash
/opt/aws/deepracer/util/otg_eth.sh
ros2 launch donkey_on_deepracer donkey_on_deepracer.launch.py
