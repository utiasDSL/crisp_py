#/usr/bin/env sh

ROS_ENV_FILE=".ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
else
    echo "$ROS_ENV_FILE not found. Using degust environment variables..."
    export ROS_DOMAIN_ID=100 
    export ROS_LOCALHOST_ONLY=0
fi

ros2 daemon stop
ros2 daemon start
