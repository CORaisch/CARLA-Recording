# README
This repo contains a collection of useful python scripts helping to record and analyse datasets from the CARLA simulator.

## Build Instructions
### Building ROS Workspace
Go into the `ros_workspace` subdirectory and build with `catkin`:
```
cd ros_workspace
catkin_make
```
Before using the rospy scripts you need to source the setup file: `source devel/setup.{bash|zsh}`.

## Dataset Generation
* Before you can generate sequences from CARLA you need to launch the CARLA server. Therefore switch into the CARLA install directory and e.g. launch the `CarlaUE4.sh` script. Most server settings will be set from my scripts. If you want to load another map, type: `CarlaUE4.sh TownXX`. For more information go to [CARLA website](http://carla.org/).
* To generate a dataset use the script `sensors_at_vehicle_sync.py`. It will spawn a vehicle at a random location in the map that is currently loaded on the server. Right now the following sensors are available:
    * RGB Camera `rgb.{left|right}`
    * Depth Camera `depth.{left|right}`
    * Semantic Segmentation Camera `semantic_segmentation.{left|right}`
* You can place as many sensors at the same place as you want. So far these sensors can be spawned at two different locations, left and right. The left position can be given with the `-left_rel_location` option. The right location is then computed from the left location and the baseline, which can be set with `-baseline` option. In this way the script ensures that it will record rectified stereo images. However it is planed to add an option that allows for arbitrary sensor settings given a .JSON file. Additionally you can capture the senor data at any framerate above 10 (this limitation comes from CARLA itself, see [this](https://carla.readthedocs.io/en/latest/configuring_the_simulation/)), but keep in mind that with a higher captuing rate the simulation itself will slow down. For more options use the help text from the script by running with `-h`.
* The script `sensors_at_vehicle_sync.py` will produce the following files:
    * `config.txt` stores useful information about the sensors used at the simulation aswell as about the simulation itself. In the following the non-obvious ones will be described:
        * `K` holds the calibration matrix for the camera sensors used at the simulation.
        * `T_left_to_right` is the transform from the left to the right camera in a stereo setting and is defined in camera space.
        * `initial absolute pose` holds the very first absolute pose with respect to the world coordinate frame of the map loaded at the simulation. It can be used to recover the absolute trajectory within this map.
    * `timestamps.txt` stores a timestamp for each frame taken of one of the cameras (right now all cameras have the same timestamp since we do not allow for asynchrone recording yet).
    * `poses_nulled.txt` stores the GT poses of the sensors transformed to the origin (i.e. with respect to T=(0,0,0) and R=I).
    * `relative_poses_nulled.txt` stores the GT poses from above but represented pairwise relative. I.e. the relative pose written in line `i` of the file is the transform from the absolute pose at `i-1` to `i` (or simply, the pose in `i` is  the camera movement from `i-1` to `i`). This representation could for example be used as labels for the training of a DNN. The relative poses are computed by left multiplying the inverse transform from `i-1` with the one from `i`, so to recover the absolute poses from the relative ones compute: ![equation](https://latex.codecogs.com/gif.latex?%28T_%7Bi-1%7D%29%5E%7B-1%7D%5Ccdot%5Chspace%7B%7D%5EiT_%7Bi-1%7D%3DT_i).

## Dataset Analysis
* to analyse the trajectories I'm using [evo](https://github.com/MichaelGrupp/evo).
* if you also want to check the orientations you can use [rviz](http://wiki.ros.org/rviz). Rviz is the visualization tool from [ROS](https://www.ros.org/). It visualizes from ROS-topics and therefore scripts are provided that converts information from the generated datasets to rosbag files. The scripts can be found at `ros_workspace/src/poses_to_bag/scripts`. In order to run the scripts follow these steps:
    1. source the `setup.{bash|zsh}` from `ros_workspace` subdirectory: `source ros_workspace/devel/setup.{bash|zsh}`
    2. run `roscore` to launch ROS server (see [ROS-tutorials](http://wiki.ros.org/ROS/Tutorials) for more information)
    3. run one of the scripts with Python 2.7. Other versions are not tested. To run `pose_to_bag_converter_complete.py` you have to pass the path to the GT poses from the CARALA dataset (`-poses`). For `pose_to_bag_converter_live.py` you additionally have to pass the path to the timestamps file (`-stamps`).
    4. to visualize with rviz you at first have to publish a transform that defines the coordinate system the trajectory will be visualized in. Type for example: `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10`, it will define the transform from `map` (rviz default frame) to `my_frame` as translation T=(0,0,0) and rotation Q=(0,0,0,1) (given as quaternion). 
    5. launch `rviz`
    6. add PoseArray display and let it listen to topic `/carla_poses` and add Path display that listens to topic `/carla_odom`.
    7. now replay the rosbag file: `rosbag play _bagfile_`. If the bagfile was converted with `pose_to_bag_converter_complete.py` you may need to add `--pause` option and skip forward by pressing `s`, else the trajectory may not appear. 

## Useful Links
- [CARALA Python API](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA C++ API](https://carla.readthedocs.io/en/latest/cpp_reference/)
