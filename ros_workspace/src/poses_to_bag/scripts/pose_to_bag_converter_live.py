#!/usr/bin/env python
import numpy as np
import argparse
import sys
import rospy
import rosbag
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray

def convert_poses(filename, poses, stamps):
    # init bag file
    bag = rosbag.Bag(filename, 'w')
    # write poses to bag
    poses_array = []
    poses_stamped_array = []
    print("poses will be written to {0}".format(filename))
    try:
        idx = 0
        while not rospy.is_shutdown() and idx < len(poses):
            # read pose from list
            pose = poses[idx] # poses are 3x4
            stamp = rospy.Time(stamps[idx])
            idx += 1
            # extract rotation
            R = np.zeros((4,4)); R[:3,:3] = pose[:3,:3]; R[3,3] = 1.0;
            t = pose[:,3].copy()
            q = tf.transformations.quaternion_from_matrix(R)
            # pack pose message
            pose_msg = Pose(Point(t[0,0], t[1,0], t[2,0]), Quaternion(q[0], q[1], q[2], q[3]))
            # pack pose-stamped message
            # read timestamps from CARLA sequence
            header_msg = Header(idx, stamp, 'map')
            pose_stamped_msg = PoseStamped(header_msg, pose_msg)
            # append poses to arrays
            poses_array.append(pose_msg)
            poses_stamped_array.append(pose_stamped_msg)
            # write to bag file
            poses_array_msg = PoseArray(header_msg, poses_array)
            path_msg = Path(header_msg, poses_stamped_array)
            bag.write('carla_poses', poses_array_msg, t=stamp)
            bag.write('carla_odom', path_msg, t=stamp)
            # give status information
            print "\r%d%% of the poses written.." % (float(idx)/float(len(poses))*100.0),
            sys.stdout.flush()
    finally:
        # give information
        print("the poses array will be published at '/carla_poses' and the path will be published to '/carla_odom'")
        bag.close()
        print("done")

if __name__ == '__main__':
    try:
        # setup argument parser
        argparser = argparse.ArgumentParser(description="Converts a recorded CARALA sequence into a ROS-bag file. It takes the GT poses and the timestamps and reconstructs the trajectory at simulation time. It outputs the poses at ROS-topic /carla_poses and the path at /carla_odom. It can be used to visualize the trajectory in RViz.")
        argparser.add_argument('--poses', '-poses', type=str, help="path to GT poses from CARLA sequence")
        argparser.add_argument('--timestamps', '-stamps', type=str, help="path to timestamps from CARLA sequence")
        args = argparser.parse_args()

        # init node
        rospy.init_node('carla_poses_node', anonymous=True)
        # read in carla poses as numpy matrices
        filename = args.poses
        f = open(filename, 'r'); lines = f.readlines(); f.close();
        poses_np = []
        for line in lines:
            l = [float(x.replace('\n', '')) for x in line.split(' ')]
            poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]]]))

        # read carla timestamps into float array
        stamps = []
        f = open(args.timestamps, 'r'); lines = f.readlines(); f.close();
        for l in lines:
            stamps.append(float(l))

        # give status information
        print("{0} poses have been parsed".format(len(poses_np)))

        # publish transformed poses
        convert_poses(filename.replace('.txt', '.bag'), poses_np, stamps)
    except rospy.ROSInterruptException:
        pass
