#!/usr/bin/python3

import numpy as np
from argparse import ArgumentParser

import rospy
from sensor_msgs.msg import JointState

from forward_kinematics import joint_angles_close

'''
This file listens to the /joint_states ROS topic and writes the results to a file
This topic is published at 50 Hz
However, if the robot is not moving, this will not continuously write the same joint pose to the
file. This is because these poses would be a waste, as this file of compiled joint states is 
intended to be used for mapping.
'''

class JointLogger:
    def __init__(self, outfile_name: str, append: bool):
        self.outfile = open(outfile_name, "a" if append else "w")
        rospy.init_node("joint_listener", anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.joint_handler)
        self.last_joint_position = None

        if not append:
            self.write_header()

    def write_header(self):
        self.outfile.write("timestamp,joint1,joint2,joint3,joint4,joint5,joint6\n")
        self.outfile.flush()

    def joint_handler(self, data: JointState):
        # if all joint angles within 0.1 degree of where they were before, ignore
        if (joint_angles_close(self.last_joint_position, data.position, 0.1 * np.pi/180)):
            return

        self.outfile.write(f"{data.header.stamp}")
        for i in range(len(data.position)):
            self.outfile.write(f",{data.position[i]}")
        self.outfile.write("\n")
        self.outfile.flush()

        self.last_joint_position = data.position

    def spin(self):
        rospy.spin()

    def cleanup(self):
        self.outfile.close()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-f", "--filename", type=str, required=True, help="Filename to Save Output To")
    parser.add_argument("-a", "--append", action="store_true", help="Whether to append")
    args = vars(parser.parse_args())
    logger = JointLogger(args["filename"], args["append"])
    logger.spin()
    logger.cleanup()
