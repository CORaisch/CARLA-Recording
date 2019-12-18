#!/bin/bash

## read input args
ABS_FILE=$1

## convert poses
ABS="${ABS_FILE%.*}"
# convert absolute to relative poses -> OUT: ${ABS}_converted.txt
python absolute_to_relative_poses.py --poses ${ABS_FILE}
# convert relative poses from matrix to euler representation -> OUT: ${ABS}_converted_converted.txt
python mat_to_euler.py --poses ${ABS}_converted.txt
# rename ${ABS}_converted_converted.txt to ${ABS}_euler.txt
mv ${ABS}_converted_converted.txt ${ABS}_euler.txt
# remove temporary files
rm ${ABS}_converted.txt

## validate poses
EULER=${ABS}_euler
# convert ${ABS}_euler.txt to matrix representation -> OUT: ${EULER}_converted.txt
python euler_to_mat.py --poses ${ABS}_euler.txt
# convert ${EULER}_converted.txt to absolute poses -> OUT: ${EULER}_converted_converted.txt
python relative_to_absolute_poses.py --poses ${EULER}_converted.txt
# rename ${EULER}_converted_converted.txt to ${EULER}_converted.txt
mv ${EULER}_converted_converted.txt ${EULER}_converted.txt
# visualize re-converted trajectory using evo (https://github.com/MichaelGrupp/evo)
evo_traj kitti -p ${ABS_FILE} ${EULER}_converted.txt
# remove temporary files
rm ${EULER}_converted.txt
