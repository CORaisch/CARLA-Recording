#!/usr/bin/env python

import sys, math, argparse
import numpy as np

def rot2euler(R):
    assert(R.shape==(3,3))
    if abs(R[2,0]) < 0.999998:
        # NOTE case: R20 != +-1 => pitch != 90º
        pitch1 = -math.asin(R[2,0])
        pitch2 = math.pi - pitch1
        roll1  = math.atan2(R[2,1]/math.cos(pitch1), R[2,2]/math.cos(pitch1))
        roll2  = math.atan2(R[2,1]/math.cos(pitch2), R[2,2]/math.cos(pitch2))
        yaw1   = math.atan2(R[1,0]/math.cos(pitch1), R[0,0]/math.cos(pitch1))
        yaw2   = math.atan2(R[1,0]/math.cos(pitch2), R[0,0]/math.cos(pitch2))
        # return the two remaining possible solutions
        return np.array([[roll1, pitch1, yaw1], [roll2, pitch2, yaw2]])
    else: # NOTE that case should not occur on our data
        print("Gimbal Lock Case!!\nTODO extend rot2euler function using prev. euler angles to determine best solution!")
        # NOTE case: Gimbal Lock since pitch==+-90º -> there are infinity many solutions !
        yaw = 0.0 # pick yaw arbitrary, since it is linked to roll
        # NOTE R20 can either be -1 or 1 in this case
        if (R[2,0] + 1.0) < 1e-6:
            # NOTE case: R20==-1
            pitch = math.pi/2.0
            roll  = yaw + math.atan2(R[0,1], R[0,2])
        else:
            # NOTE case: R20==1
            pitch = -math.pi/2.0
            roll  = -yaw + math.atan2(-R[0,1], -R[0,2])
        # return one sample solution in the gimbal lock case
        return np.array([[roll, pitch, yaw]])

def pose2str(p):
    t  = p[:3, 3]
    Re = rot2euler(p[:3, :3])[0]
    return str(t[0,0]) + " " + str(t[1,0]) + " " + str(t[2,0]) + " " + str(Re[0]) + " " + str(Re[1]) + " " + str(Re[2]) + "\n"

def main():
    # setup argparser
    argparser = argparse.ArgumentParser(description="Converts poses in matrix form to euler poses.")
    argparser.add_argument('--poses', '-p', type=str, help="path to GT pose matrices")
    args = argparser.parse_args()

    # read in poses as numpy matrices
    print("[INFO] parsing poses...", end='', flush=True)
    filename = args.poses
    f = open(filename, 'r'); lines = f.readlines(); f.close();
    poses_np = []
    for line in lines:
        l = [float(x.replace('\n', '')) for x in line.split(' ')]
        poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]], [0.0, 0.0, 0.0, 1.0]]))
    print(" done")

    # give status information
    print("[INFO] {} poses have been parsed".format(len(poses_np)))

    # iterate poses and serialize to string
    print("[INFO] converting poses from matrices to euler...", end='', flush=True)
    poses_str = pose2str(poses_np.pop(0).copy())
    for pose in poses_np:
        poses_str += pose2str(pose)
    print(" done")

    # write absolute poses to file
    filename_out = filename.replace('.txt', '_converted.txt')
    print("[INFO] writing matrix poses to file at '{}'...".format(filename_out), end='', flush=True)
    with open(filename_out, 'w') as poses_file:
        poses_file.write(poses_str)
    print(" done")

if __name__ == "__main__":
    main()
