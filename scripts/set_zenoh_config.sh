export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/10.157.163.20:7447"]'
export RUST_LOG=zenoh=info

ros2 daemon stop && ros2 daemon start
