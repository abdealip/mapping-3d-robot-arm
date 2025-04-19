# Mapping Framework for a Robot Manipulator

The code in this repository is used to build a 3D map of the workspace of a robot arm.

There are three main high-level functions:
- Mapping the space taken up by the robot links as free space
- Using an intel realsense depth camera to map occupied space around the robot as a point cloud
- Registering a pre-built 3D environment to the point cloud

## Dependencies

### Setting up ROS

As this was developed on ubuntu 20.04, all the ROS-related tools were tested using ROS noetic.

1. Install ROS using the steps on the wiki: https://wiki.ros.org/noetic/Installation/Ubuntu
2. Create a directory for a catkin workspace, e.g. `mkdir -p mapping_robot_arm_ws/src`
2. Clone the khi_robot repo: https://github.com/Kawasaki-Robotics/khi_robot in the src directory of the catkin workspace: `cd mapping_robot_arm_ws/src && git clone https://github.com/Kawasaki-Robotics/khi_robot`
3. Build the catkin workspace using `cd mapping_robot_arm_ws && catkin build`
4. Add `source mapping_robot_arm_ws/devel/setup.bash` to your `.bashrc` file. This can be done with `echo source /path/to/catkin/workspace/devel/setup.bash >> ~/.bashrc`. If you have other catkin workspaces on your machine, do not do this step. Just manually run `source mapping_robot_arm_ws/devel/setup.bash` every time you want to use this workspace

I personally did `sudo apt-get install ros*controller*`, because I was getting an error that "JointStateTrajectory Controlloer does not exist", although that is probably overkill. I was following this stackexchange page: https://robotics.stackexchange.com/questions/78821/gazebo-could-not-load-controller-jointtrajectorycontroller-does-not-exist-mas. You probably only need to install a small subset of these controllers for the khi robot packages to work.

### Python Packages

This repo was developed using python 3.10 on ubuntu 20.04

Packages used:
- rospy (`sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`)
- numpy (`pip install numpy`) - typically automatically installed by other packages
- matplotlib (`pip install matplotlib`)
- numpy-stl (`pip install numpy-stl`)
- pyrealsense2 (`pip install pyrealsense2`)
- cv2 (I installed this a long time ago so I don't remember how I did it, but probably `pip install opencv-python`)
- pillow (`pip install pillow`)
- pandas (`pip install pandas`)

## Hardware Interfaces

The work in this repo used the RS007L robot manipulator and the D435 intel realsense depth camera.

### Interfacing with the RS007L

Setting up your netplan:

Run `ifconfig` or `ip addr` to determine the name(s) of the ethernet interface(s) your machine has.

You must enable DHCP for your ethernet interface. To do this, I had to edit `/etc/netplan/01-network-manager-all.yaml` to look like the following:

```
network:
  version: 2
  renderer: networkd
  ethernets:
    enp6s0:
      dhcp4: true
```

On my machine, the ethernet interface name was `enp6s0`. This will be different for different machines. After editing the netplan, run `sudo netplan apply`

Navigate the teachpendant to find the IP address of the RS007L.

Alternatively, use `nmap -sL 192.168.50.0/24` while connected to the robot's subnet to scan the IP addresses while connected to the network, disconnect the robot's ethernet, run the `nmap` command again, and determine which IP address is missing - that will be the IP address of the RS007L. If your subnet mask is different, you will have to use the IP address corresponding to the subnet instead of `192.168.50.0/24`.

Change the `ip` arguments in `tools/rs007l_listen.sh` and `khi_robot/khi_robot_bringup/launch/rs007l_bringup.launch`

In order to connect to the robot and receive feedback, the RS007L must be powered on and in repeat mode. In addition, the ethernet cable connected to the network switch/router must be plugged into your machine. Run `ifconfig` or `ip addr` and verify your ethernet interface has an IP address consistent with the subnet (e.g 192.168.50.*) before continuing.

In order to control the robot in rviz, you must ensure the RS007L is in "STEP CONT", not "STEP ONCE" on the teach pendant. However, this project does not necessarily need to control the robot in rviz.

In order to verify the connection to the robot, run `tools/rs007l_listen.sh`, then wait a few seconds. In another terminal, run `src/joint_state_capture.py` to see the joint positions and velocities printed out at 10 Hz. This is simply downsampling the data on the `/joint_states` ROS topic.

### Interfacing with the Intel Realsense D435 Camera

The documentation for the D435 camera can be found here: https://www.intelrealsense.com/depth-camera-d435/

#### Physical Mounting

The depth camera should be fastened to the end-effector of the RS007L. This was done using 3d-printed parts (models/end_effector_adaptor.stl and models/D435_camera_end_effector_mount.stl). I had to manually adjust the hole diameters physically because they were too small after they were 3D printed for the bolts to go through them. You may want to expand the diameter of all holes in the CAD model by 1 mm.

Ensure when mounting the camera, that it is right side up in the RS007L's end effector frame - this is evident because there is one hole in the RS007L end effector (out of eight) which does not have threads - that should align with the top of the camera.

#### Communication

To connect the camera to your machine, simply use the provided USB cable. You may want to use a USB extender as the cable is only a few feet long.

To test the connection, run `src/camera.py`. An plot showing the depth measurements and RGB image should show up. If the script exits with the error "No device connected", then the USB cable is not plugged in properly. If the script exits with the error "Did not receive frames after 5 seconds", try again until the script works.

## Free Space Mapping using Robot Links

The `src/offline_mapper.py` and `src/online_mapper.py` programs are used to map the free space occupied by the robot links.

The `config/map_config.json` specifies the parameters, including:
- The position and orientation of each cylinder in the robot's reference configuration
- The base point, axis, and radius of each cylinder
- The joint twists of the robot (each row is a lowered se(3) twist)
- The workspace of the robot (axis limits)
- The resolution of the map (voxel size)
- Sampling parameters for robot joint states

The online mapper will subscribe to the `/joint_states` ros topic and save the result to a map file when the user exits.

The offline mapper consumes a CSV file of logged joint states and save the result to a map file.

To obtain a log of joint states, run the `src/joint_state_logger.py` program and specify an output file. This will be the input file to `src/offline_mapper.py`.

## Occupied Space Mapping using a Point Cloud

The `src/camera_mapper.py` is used to map occupied space using a point cloud.

Process to build a map using live data:
1. Install camera on robot
2. Connect laptop to lab network
3. Connect laptop to camera (run `src/camera.py` to test connection, keep trying until connection is confirmed)
4. Turn on robot, ensure it is in repeat mode, turn motor on, load program `map_demo`
5. Confirm lab ethernet by running ifconfig and ensuring the ethernet interface has an IP address matching $ROS_IP (192.168.50.something)
6. Run rs007l_listen.sh after lab ethernet connection is confirmed
7. Run python program `src/camera_mapper.py -o <output_directory>`, replacing output_directory with the directory where you want to store the camera data and the resulting map.
8. Hit "Cycle Start" on the teachpendant
9. For each robot waypoint:
  1. When robot reaches next pose and stops, the operator hits Enter in the `src/camera_mapper.py` terminal to perform a capture
  2. Operator hits "Go" on the teach pendant to make the robot move to the next pose
10. operator types "exit" in the python program terminal - program will write the map to a file and terminate

It is also acceptable to manually jog the robot with the teachpendant rather than using a pre-existing program. Note that the robot must be stationary when performing a capture in step 9.1.

Process to build a map after having recorded live data:
- Run `src/camera_mapper.py -i <input_directory>` where input_directory matches the output_directory supplied to the camera mapper in step 7 above, when building the map from live data.

## Registering a Pre-Built 3D Environment to a Point Cloud

In order to register an existing 3D environment, The pre-existing environment must have the following format. Obstacles to be registered to the point cloud must have their own STL models in the `models` directory. Then, the json file `config/environment.json` must be adjusted to describe the pose of those STL models in the robot's base frame. Then, follow these steps:
1. Run `src/environment_rasterized_viewer -o <output_dir>` where `output_dir` is the directory to store the point cloud representation of each obstacle.
2. Run `src/register_environment_to_map -i <input_dir> -m <camera_map_file> -o <output_json> -p <output_point_cloud_dir>`, where `input_dir` is the same as `output_dir` from step 1, `camera_map_file` is the path to the camera map to register this environment against, `output_json` is the json file to write the resulting environment configuration to with the corrected obstacle poses, and `output_point_cloud_dir` is the directory to which the corrected point clouds should be stored
3. Run `src/stl_combine.py -e <corrected_environment_json_path> -o <output_stl_path>`, where `corrected_environment_json_path` is the same as `output_json` from step 2, and `output_stl_path` is the path to which to save the resulting STL file representing the corrected environment.

### Visualizing the Corrected Environment in Rviz

To control the robot and visualize the corrected environment in rviz (with collision detection and collision-aware path planning), do the following:
1. Connect to the lab network (verify connection with `ifconfig`or `ip addr`)
2. Power on RS007L and ensure it is in repeat mode
3. Run `roscore` in one terminal
4. Run `roslaunch khi_robot_bringup rs007l_bringup.launch ip:=192.168.50.11` in another terminal after roscore is up and running
5. Run `roslaunch khi_rs007l_moveit_config moveit_planning_execution.launch` in another terminal, this should launch RVIZ and the robot should be visible
6. Run `./src/rviz_load_stl.py -m <path_to_stl>` where `path_to_stl` is the same as `output_stl_path` in step 3 of "Registering a Pre-Built 3D Environment to a Point Cloud". This should make the environment visible in RVIZ, and the surfaces in it will now be used for collision detection and collision aware motion planning.

## Summary of all executables

All python executables in the `src` of this repo can be ran with `-h` or `--help` to get a description of the required and optional arguments the user can provide. This is a list of the executables and what they do:
- `src/camera_mapper.py` - build a map using camera data
- `src/camera.py` - test connection to camera and visualize camera data
- `src/compare_environment_to_point_cloud.py` - compares a rasterized environment point cloud to a camera map visually, and prints the mean registration error
- `src/environment_rasterized_viewer.py` - create point clouds from CAD models and their poses
- `src/joint_state_capture.py` - test connection to RS007L robot and print current joint states at 10 Hz
- `src/joint_state_logger.py` - capture joint states of RS007L robot and write to a CSV file
- `src/map_fidelity.py` - get statistics for the fidelity of freespace maps
- `src/map_visualizer.py` - visualize the free space of a freespace map, or the occupied space of a point cloud map
- `src/offline_mapper.py` - build a freespace map from a log of joint poses, optionally output plot images periodically to be stitched into a video
- `src/online_mapper.py` - build a freespace map from live joint state data
- `src/point_cloud_rasterizer.py` - rasterize raw point cloud data (a list of xyz coordinates) into a gridmap and save it
- `src/range_image_to_point_cloud.py` - create a point cloud from a depth image and the joint states
- `src/register_environment_to_map.py` - register a json-specified environment to a camera map
- `src/rviz_load_stl.py` - add an STL file representing an obstacle to the environment in RVIZ
- `src/stl_combine.py` - create a single STL by combining a list of STL's and their poses
- `src/stl_util.py` - visualize how a point cloud is created from an STL file
- `src/video_stitcher.py` - combine the images output from `offline_mapper.py` (if `video-dir` was specified) into a video
- `tools/rs007l_listen.sh` - a shell script to combine some ROS commands necessary to set up the publication of the `/joint_states` channel from the RS007L (this is not a python script - there are no arguments and there is no `--help`)
