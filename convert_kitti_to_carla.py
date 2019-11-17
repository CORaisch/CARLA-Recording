#!/usr/bin/env python

import sys, math
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
    R = T[:3, :3]; t_k = T[:3, 3];
    T_inv[:3, :3] = R.T
    T_inv[:3,  3] = -R.T * t_k
    return T_inv

def c(x):
    return math.cos(x)

def s(x):
    return math.sin(x)

def R_x(a):
    return np.matrix([[1.0, 0.0, 0.0   ],
                      [0.0, c(a), -s(a)],
                      [0.0, s(a), c(a) ]])

def R_y(a):
    return np.matrix([[ c(a), 0.0, s(a)  ],
                      [ 0.0,  1.0, 0.0   ],
                      [-s(a), 0.0, c(a)  ]])

def R_z(a):
    return np.matrix([[c(a), -s(a), 0.0],
                      [s(a),  c(a), 0.0],
                      [0.0,   0.0,  1.0]])

def main():
    # setup argparser
    argparser = argparse.ArgumentParser(description="Converts poses from the coordinate system of KITTI to CARLA.")
    argparser.add_argument('--poses', '-p', type=str, help="path to poses")
    args = argparser.parse_args()

    # read inposes as numpy matrices
    filename = args.poses
    f = open(filename, 'r'); lines = f.readlines(); f.close();
    poses_np = []
    for line in lines:
        l = [float(x.replace('\n', '')) for x in line.split(' ')]
        poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]], [0.0, 0.0, 0.0, 1.0]]))

    # give status information
    print("{0} poses have been parsed".format(len(poses_np)))

    # iterate and convert each pose
    poses_str = ""
    for pose in poses_np:
        ## get KITTI rotation (R_k) and translation (t_k)
        R_k = pose[:3,:3]; t_k = pose[:3,3];
        ## convert translation: x_c -> z_k | y_c -> -x_k | z_c -> -y_k
        t_c = t_k.copy()
        t_c[0,0] = t_k[2,0]; t_c[1,0] = -t_k[0,0]; t_c[2,0] = -t_k[1,0];
        ## convert rotation
        # define R kitti to carla: 1) rotate 90º about world (carla) y axis, 2) rotate -90º about world(carla) x axis
        R_k_to_c = R_x(-math.pi/2.0) * R_y(math.pi/2.0)
        # transform kitti rotations into carla coordinate system
        R_c = R_k_to_c * R_k * R_k_to_c.T
        # compose final pose
        T_c = pose.copy(); T_c[:3,:3] = R_c; T_c[:3,3] = t_c;
        poses_str += mat_to_string(T_c)

    # write converted poses to file
    filename_out = filename.replace('.txt', '_carla.txt')
    with open(filename_out, 'w') as abs_poses_file:
        abs_poses_file.write(poses_str)

    # give status report
    print("converted poses have been written to '{0}'".format(filename_out))

if __name__ == "__main__":
    main()
