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

def invert_pose(T):
    T_inv = T.copy()
    R = T[:3, :3]; t = T[:3, 3];
    T_inv[:3, :3] = R.T
    T_inv[:3,  3] = -R.T * t
    return T_inv

def main():
    # setup argparser
    argparser = argparse.ArgumentParser(description="Converts absolute poses to relative poses.")
    argparser.add_argument('--poses', '-p', type=str, help="path to absolute poses")
    args = argparser.parse_args()

    # read in carla poses as numpy matrices
    filename = args.poses
    f = open(filename, 'r'); lines = f.readlines(); f.close();
    poses_np = []
    for line in lines:
        l = [float(x.replace('\n', '')) for x in line.split(' ')]
        poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]], [0.0, 0.0, 0.0, 1.0]]))

    # give status information
    print("{0} absolute poses have been parsed".format(len(poses_np)))

    # iterate absolute poses and concatenate relative poses for each frame
    T_last = poses_np.pop(0).copy()
    abs_poses_str = mat_to_string(T_last)
    for pose in poses_np:
        # invert last pose
        T_last_inv = invert_pose(T_last)
        T_rel = T_last_inv * pose
        abs_poses_str += mat_to_string(T_rel)
        T_last = pose.copy()

    # write relative poses to file
    filename_out = filename.replace('.txt', '_converted.txt')
    with open(filename_out, 'w') as abs_poses_file:
        abs_poses_file.write(abs_poses_str)

    # give status report
    print("converted absoulute poses written to '{0}'".format(filename_out))

if __name__ == "__main__":
    main()