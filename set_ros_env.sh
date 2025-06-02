#/usr/bin/env sh

ROS_ENV_FILE=".ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
else
    echo "$ROS_ENV_FILE not found. Using degust environment variables..."
    export ROS_DOMAIN_ID=100 
    export ROS_LOCALHOST_ONLY=0

    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CONFIG_URI=$HOME/repos/crisp_py/config/rmw/humble_zenoh_router_config.json5

    # export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # export CYCLONEDDS_URI=file:///home/lsy_franka/repos/crisp_py/config/rmw/cyclone_config.xml
fi

ros2 daemon stop
ros2 daemon start
