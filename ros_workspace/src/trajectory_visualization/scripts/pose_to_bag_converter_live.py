#!/usr/bin/python2
import numpy as np
import argparse, sys, os, rospy, rosbag, tf
from PIL import Image
from std_msgs.msg import Header
from nav_msgs.msg import Path
from sensor_msgs.msg import Image as Image_msg
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray

cityscapes_pallete = {
    0: (0,0,0), 1: (70,70,70), 2: (100,40,40), 3: (55,90,80), 4: (220,20,60), 5: (153,153,153), 6: (157,234,50), 7: (128,64,128), 8: (244,35,232), 9: (107,142,35), 10: (0,0,142), 11: (102,102,156), 12: (220,220,0),
    13: (70,130,180), 14: (81,0,81), 15: (150,100,100), 16: (230,150,140), 17: (180,165,180), 18: (250,170,30), 19: (110,190,160), 20: (170,120,50), 21: (45,60,150), 22: (145,170,100)
}

def pack_image(base, index, stamp, s):
    # prepare Image message
    msg = Image_msg()
    msg.header.stamp = stamp
    msg.encoding = "rgb8"
    msg.header.frame_id = "map"
    # load image
    path = os.path.join(base, s[0], s[1], '{}.png'.format(index))
    img = Image.open(path).convert('RGB')
    msg.width = img.width
    msg.height = img.height
    # postprocess semseg and depth data
    if s[0] == 'rgb':
        raw = list(img.getdata())
    elif s[0] == 'semantic_segmentation':
        raw = [cityscapes_pallete[p[0]] for p in img.getdata()]
    elif s[0] == 'depth':
        ppd = np.asarray([(float((p[0]+p[1]*256+p[2]*256**2))/float(256**3-1))*1000.0 for p in img.getdata()]) # compute per pixel dists
        ppd = np.log(ppd) # apply logarithmic map
        ppd = (ppd-np.min(ppd))*255/(np.max(ppd)-np.min(ppd)) # scale to [0,255]
        raw = [(p,p,p) for p in ppd.astype(np.uint8)]
    # pack image data into message
    msg.data = [pix for pixdata in raw for pix in pixdata]
    return msg

def convert_poses(base, poses, stamps, sensors):
    # init bag file
    filename = os.path.join(base, 'poses.bag')
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
            im_index = str(idx).zfill(6) # zero pad to 6 digits by convention
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
            # write sensor measurements
            for sensor in sensors:
                s = sensor.split('.')
                im_msg = pack_image(base, im_index, stamp, s)
                bag.write('{}_{}'.format(s[0],s[1]), im_msg, t=stamp)
            # give status information
            print "\r%d%% of the poses written.." % (float(idx)/float(len(poses))*100.0),
            sys.stdout.flush()
    finally:
        # give information
        print("\nthe poses array will be published at '/carla_poses' and the path will be published to '/carla_odom'")
        bag.close()
        print("done")

if __name__ == '__main__':
    try:
        # setup argument parser
        argparser = argparse.ArgumentParser(description="Converts a recorded CARALA sequence into a ROS-bag file. It takes the GT poses and the timestamps and reconstructs the trajectory at simulation time. It outputs the poses at ROS-topic /carla_poses and the path at /carla_odom. It can be used to visualize the trajectory in RViz.")
        argparser.add_argument('--poses', '-poses', type=str, help="path to GT poses from CARLA sequence")
        argparser.add_argument('--timestamps', '-stamps', type=str, help="path to timestamps from CARLA sequence")
        argparser.add_argument('--sensors', '-sensors', type=str, default="", help="string of sensor measurements which should additionally be packed into rosbag file. Sample: \"rgb.left depth.left semantic_segmentation.left\".")
        args, unknown = argparser.parse_known_args() # ignore args added by roslaunch

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
        convert_poses(os.path.split(filename)[0], poses_np, stamps, args.sensors.split(','))
    except rospy.ROSInterruptException:
        pass
