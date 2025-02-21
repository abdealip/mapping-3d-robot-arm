#!/usr/bin/python3

import rospy
import time
import numpy as np

from sensor_msgs.msg import JointState

def joint_angles_close(position1, position2, threshold):
    if position1 == None or position2 == None:
        return False
    
    if (len(position1) != len(position2)):
        return False
    
    for i in range(len(position1)):
        if abs(position1[i] - position2[i]) > threshold:
            return False
    
    return True

class JointRecorder:
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
    jr = JointRecorder("joint_states.csv", False)
    jr.spin()
    jr.cleanup()
