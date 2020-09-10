#!/bin/bash

# set current working directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# setup ROS environment
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${DIR}/ros_workspace/src

# process arguments
if [ ${#} -ne 3 ]
then
   # incorrect number of arguments passed
   echo "you must either pass the GT .txt file or the already converted .bag file as an argument"
else
    # POSES == path to poses
    POSES=${1}
    # STAMPS == path to timestamps
    STAMPS=${2}
    # SENSORS == additional sensors to pack into rosbag
    SENSORS=${3}
    # check if path of POSES is already absolute
    if [ ${POSES: 0: 1} != "/" ]
    then
        # if relative path is given concat to absolute
        POSES=${DIR}/${POSES}
    fi
    # check if path of STAMPS is already absolute
    if [ ${STAMPS: 0: 1} != "/" ]
    then
        # if relative path is given concat to absolute
        STAMPS=${DIR}/${STAMPS}
    fi
    # check if input poses is bag or txt file
    if [ ${POSES: -4} == ".txt" ]
    then
        # convert GT poses to bagfile
        echo "convert GT poses to bagfile ..."
        roslaunch ${DIR}/ros_workspace/src/trajectory_visualization/launch/convert_trajectory_live.launch poses_file:=${POSES} timestamps_file:=${STAMPS} sensors_str:="${SENSORS}"
        BAGFILE=${POSES: 0: -4}.bag
        echo "done."
    elif [ ${POSES: -4} == ".bag" ]
    then
        # set bagfile
        BAGFILE=${POSES}
    else
        # incorrect argument passed
        echo "you must either pass the GT .txt file or the already converted .bag file as an argument"
    fi
fi

# launch visualization nodes
echo "start rviz"
roslaunch  ${DIR}/ros_workspace/src/trajectory_visualization/launch/visualize_trajectory.launch rosbag_file:="${BAGFILE}" rviz_config_file:="${DIR}/utils/viz_trajectory.rviz"
