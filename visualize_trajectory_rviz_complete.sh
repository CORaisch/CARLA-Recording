#!/bin/bash

# set current working directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# setup ROS environment
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${DIR}/ros_workspace/src

# process arguments
if [ ${#} -ne 1 ]
then
   # incorrect number of arguments passed
   echo "you must either pass the GT .txt file or the already converted .bag file as an argument"
else
    # set path to file
    ARG=${1}
    # check if path is already absolute
    if [ ${ARG: 0: 1} != "/" ]
    then
        # if relative path is given concat to absolute
        ARG=${DIR}/${ARG}
    fi
    # check if input is bag or txt file
    if [ ${ARG: -4} == ".txt" ]
    then
        # convert GT poses to bagfile
        echo "convert GT poses to bagfile ..."
        roslaunch ${DIR}/ros_workspace/src/trajectory_visualization/launch/convert_trajectory_complete.launch poses_file:=${ARG}
        BAGFILE=${ARG: 0: -4}.bag
        echo "done."
    elif [ ${ARG: -4} == ".bag" ]
    then
        # set bagfile
        BAGFILE=${ARG}
    else
        # incorrect argument passed
        echo "you must either pass the GT .txt file or the already converted .bag file as an argument"
    fi
fi

# launch visualization nodes
echo "start rviz"
roslaunch  ${DIR}/ros_workspace/src/trajectory_visualization/launch/visualize_trajectory.launch rosbag_file:="${BAGFILE} -d 1" rviz_config_file:="${DIR}/utils/viz_trajectory.rviz"
