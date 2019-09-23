#!/bin/bash

# TODO check input argument

# extract sequence base directory
BASE_DIR=$1

## transform euler poses to absolute pose matrices
python euler_to_mat.py --poses $BASE_DIR/euler_poses_nulled.txt

## transform relative euler poses to absolute pose matrices
# transform relative euler poses to relative pose matrices
python euler_to_mat.py --poses $BASE_DIR/relative_euler_poses_nulled.txt
# transform relative pose matrices to absolute pose matrices
python relative_to_absolute_poses.py --poses $BASE_DIR/relative_euler_poses_nulled_converted.txt

## transform relative pose matrices to absolute pose matrices
python relative_to_absolute_poses.py --poses $BASE_DIR/relative_poses_nulled.txt

# call evo to visualize all GT formats (for debugging/validation)
evo_traj kitti -p $BASE_DIR/relative_euler_poses_nulled_converted_converted.txt $BASE_DIR/euler_poses_nulled_converted.txt $BASE_DIR/relative_poses_nulled_converted.txt $BASE_DIR/poses_nulled.txt
