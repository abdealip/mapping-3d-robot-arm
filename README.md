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

On my machine, the ethernet interface name was `enp6s0`. This will be different for different machines.

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



## Occupied Space Mapping using a Point Cloud



## Registering a Pre-Built 3D Environment to a Point Cloud

### Visualizing the Transformed Environment in Rviz

## Visualization Tools
