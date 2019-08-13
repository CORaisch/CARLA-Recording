#!/usr/bin/python2
import numpy as np
import argparse
import sys
import rospy
import rosbag
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray

def convert_poses(filename, poses):
    # init bag file
    rospy.init_node('carla_poses_node', anonymous=True)
    bag = rosbag.Bag(filename, 'w')
    # make header for final PoseArray message
    header = Header(0, rospy.Time.now(), 'map')
    # write poses to bag
    poses_array = []
    poses_stamped_array = []
    print("poses will be written to {0}".format(filename))
    try:
        idx = 0
        while not rospy.is_shutdown() and idx < len(poses):
            # read pose from list
            pose = poses[idx] # poses are 3x4
            idx += 1
            # extract rotation
            R = np.zeros((4,4)); R[:3,:3] = pose[:3,:3]; R[3,3] = 1.0;
            t = pose[:,3].copy()
            q = tf.transformations.quaternion_from_matrix(R)
            # pack pose message
            pose_msg = Pose(Point(t[0,0], t[1,0], t[2,0]), Quaternion(q[0], q[1], q[2], q[3]))
            # pack pose-stamped message
            header_msg = Header(idx, rospy.Time.now(), 'map')
            pose_stamped_msg = PoseStamped(header_msg, pose_msg)
            # append poses to arrays
            poses_array.append(pose_msg)
            poses_stamped_array.append(pose_stamped_msg)
            # give status information
            print "\r%d%% of the poses written.." % (float(idx)/float(len(poses))*100.0),
            sys.stdout.flush()
    finally:
        # give information
        print("\nthe poses array will be published at '/carla_poses' and the path will be published to '/carla_odom'")
        # write to bag file
        poses_array_msg = PoseArray(header, poses_array)
        path_msg = Path(header, poses_stamped_array)
        bag.write('carla_poses', poses_array_msg)
        bag.write('carla_odom', path_msg)
        bag.close()
        print("done")

if __name__ == '__main__':
    try:
        # setup argument parser
        argparser = argparse.ArgumentParser(description="Converts a recorded CARALA sequence into a ROS-bag file. It takes the GT poses and reconstructs the trajectory at once. It outputs the poses at ROS-topic /carla_poses and the path at /carla_odom. It can be used to visualize the trajectory in RViz.")
        argparser.add_argument('--poses', '-poses', type=str, help="path to GT poses from CARLA sequence")
        args, unknown = argparser.parse_known_args() # ignore args added by roslaunch

        # read in carla poses as numpy matrices
        filename = args.poses
        f = open(filename, 'r'); lines = f.readlines(); f.close();
        poses_np = []
        for line in lines:
            l = [float(x.replace('\n', '')) for x in line.split(' ')]
            poses_np.append(np.matrix([[l[0], l[1], l[2], l[3]], [l[4], l[5], l[6], l[7]], [l[8], l[9], l[10], l[11]]]))

        # give status information
        print("{0} poses have been parsed".format(len(poses_np)))

        # publish transformed poses
        convert_poses(filename.replace('.txt', '.bag'), poses_np)
    except rospy.ROSInterruptException:
        pass
