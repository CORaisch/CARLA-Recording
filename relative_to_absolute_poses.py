#!/usr/bin/env python

import sys
import argparse
import numpy as np

def mat_to_string(mat):
    # NOTE expecting 3x4 mat
    ret  = str(mat[0,0]) + " " + str(mat[0,1]) + " " + str(mat[0,2]) + " " + str(mat[0,3]) + " "
    ret += str(mat[1,0]) + " " + str(mat[1,1]) + " " + str(mat[1,2]) + " " + str(mat[1,3]) + " "
    ret += str(mat[2,0]) + " " + str(mat[2,1]) + " " + str(mat[2,2]) + " " + str(mat[2,3]) + "\n"
    return ret

def main():
    # setup argparser
    argparser = argparse.ArgumentParser(description="Converts recorded relative poses from a CARLA sequence into absolute poses. This can be used for validating the relative GT poses generated by the CARLA simulator.")
    argparser.add_argument('--poses', '-poses', type=str, help="path to relative GT poses from CARLA sequence")
    args = argparser.parse_args()

    # read in carla poses as numpy matrices
    filename = args.poses
    f = open(filename, 'r'); lines = f.readlines(); f.close();
    poses_np = []
    for line in lines:
        l = [float(x.replace('\n', '')) for x in line.split(' ')]
        poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]], [0.0, 0.0, 0.0, 1.0]]))

    # give status information
    print("{0} relative poses have been parsed".format(len(poses_np)))

    # iterate relative poses and concatenate absolute pose for each frame
    T_last = poses_np.pop(0).copy()
    abs_poses_str = mat_to_string(T_last)
    for pose in poses_np:
        T_i = T_last * pose
        abs_poses_str += mat_to_string(T_i)
        T_last = T_i.copy()

    # write absolute poses to file
    filename_out = filename.replace('.txt', '_converted.txt')
    with open(filename_out, 'w') as abs_poses_file:
        abs_poses_file.write(abs_poses_str)

    # give status report
    print("converted absoulute poses written to '{0}'".format(filename_out))

if __name__ == "__main__":
    main()