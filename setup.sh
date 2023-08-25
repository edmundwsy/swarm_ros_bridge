#!/bin/bash
source /home/drone/ws/catkin_ws/devel/setup.bash

# linear controller
# source /home/drone1/ws/linear_px4ctrl/devel/setup.bash

# realsense
# source /home/drone1/ws/rs/devel/setup.bash

alias mavros="source ./mavros.sh"
alias comm="roslaunch swarm_ros_bridge onboard_bridge.launch config:=onboard_plan.yaml"
alias comm_track="roslaunch swarm_ros_bridge onboard_bridge.launch config:=onboard_track.yaml"
alias plan="roslaunch plan_manager real_drone_nodelet.launch"
alias ctrl="source /home/drone/ws/linear_px4ctrl/devel/setup.bash && 
            roslaunch px4ctrl run_ctrl.launch"
alias depth="source /home/drone/ws/rs/devel/setup.bash &&
            roslaunch realsense2_camera rs_d450.launch"