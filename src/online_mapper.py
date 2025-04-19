#!/usr/bin/python3

import os
import numpy as np
from argparse import ArgumentParser

import rospy
from sensor_msgs.msg import JointState

from mapper import Mapper

'''
This file listens to the /joint_states ROS topic and writes the results to a file
This topic is published at 50 Hz
However, if the robot is not moving, this will not continuously write the same joint pose to the
file. This is because these poses would be a waste, as this file of compiled joint states is 
intended to be used for mapping.
'''

def joint_angles_close(position1, position2, threshold):
    if position1 == None or position2 == None:
        return False
    
    if (len(position1) != len(position2)):
        return False
    
    for i in range(len(position1)):
        if abs(position1[i] - position2[i]) > threshold:
            return False
    
    return True

class OnlineMapper(Mapper):
    def __init__(self, json_file, sample_period_seconds, do_live_display=False):
        super().__init__(json_file, do_live_display=do_live_display)

        rospy.init_node("joint_listener", anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.joint_handler)
        self.last_joint_position = None
        self.last_joint_time = 0
        self.sample_period = sample_period_seconds

    def joint_handler(self, data: JointState):
        # if all joint angles within 0.1 degree of where they were before, ignore
        if (joint_angles_close(self.last_joint_position, data.position, 0.1 * np.pi/180)):
            return

        timestamp = data.header.stamp.to_sec()
        if timestamp - self.last_joint_time < self.sample_period:
            return

        self.consume_joint_angles(data.position)
        self.last_joint_position = data.position
        self.last_joint_time = timestamp

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser = ArgumentParser()
    parser.add_argument("-m", "--map-file", type=str, required=True, help="Filename to Save Output To")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")
    parser.add_argument("-s", "--sample-time", type=float, default=0.1, help="Sample Time of Joint States in Seconds")
    parser.add_argument("--live-display", action="store_true", help="Whether to do a live display of the map updating")
    args = vars(parser.parse_args())

    m = OnlineMapper(args["config"], args["sample_time"], args["live_display"])
    m.spin()
    m.write_to_file(args["map_file"])
