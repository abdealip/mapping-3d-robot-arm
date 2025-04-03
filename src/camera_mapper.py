#!/usr/bin/python3

import os
import json
from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
import rospy

from plotter_3d import *
from camera_joint_snapshot import CameraJointTracker
from forward_kinematics import Twist, ForwardKinematics
from range_image_to_point_cloud import RangeImagePointCloud

class CameraMapper:
    def __init__(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        twists = [Twist(twist) for twist in config["joint_twists"]]
        links = config["mapping_links"]
        for link in links:
            if link["type"] == "camera":
                reference_config = np.array(link["reference_config"])
                break

        fk = ForwardKinematics(twists, reference_config)
        self.ripc = RangeImagePointCloud(fk)

        self.points = np.empty([0, 3])

    def process_snapshot(self, depth_image, joint_states):
        points = self.ripc.point_cloud_from_range_image(depth_image, joint_states)
        self.points = np.row_stack([self.points, points])
        return points

    def process_snapshot_from_dir(self, input_dir):
        depth_image = np.loadtxt(os.path.join(input_dir, "depth.txt"))
        joint_states = np.loadtxt(os.path.join(input_dir, "joints.txt"))
        return self.process_snapshot(depth_image, joint_states)

    def write_point_cloud_to_file(self, filename):
        np.savetxt(filename, self.points)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", default=None, help="Input Directory with Depth Image and Joint States")
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")
    parser.add_argument("-o", "--output", default=None, help="Input Directory with Depth Image and Joint States")

    options = parser.parse_args()

    cm = CameraMapper(options.config)

    if options.input != None:
        cm.process_snapshot(options.input)

        plotter = Plotter3D(ViewEnum.ISO, "Test", [-5, 5], [-5, 5], [-5, 5], interactive=False)

        plotter.add_points(cm.points, "Depth Points")
        plotter.update()

        print(len(cm.points))
        plt.savefig("test.png")
        plt.show()

    else:
        cjt = CameraJointTracker()
        point_cloud_plotter = Plotter3D(ViewEnum.ISO_BACK, "Point Cloud", [-2, 2], [-2, 2], [-2, 2], figsize=[7, 7])
        fig = plt.figure(constrained_layout=True, figsize=(7, 4))
        while not rospy.is_shutdown():
            user = input("Press Enter to capture a new frame, exit to exit: ")
            if user == "exit":
                rospy.signal_shutdown("User")
                break
            color_image, depth_image, joint_state = cjt.snapshot_with_color()
            fig.add_subplot(2, 1, 1)
            cjt.camera.plot_depth_image(depth_image, fig)
            fig.add_subplot(2, 1, 2)
            cjt.camera.plot_color_image(color_image, fig)

            fig.canvas.draw()
            fig.canvas.flush_events()

            new_points = cm.process_snapshot(depth_image, joint_state)
            print(f"Added {len(new_points)} points")
            point_cloud_plotter.add_points(new_points, "Depth Points")

        cm.write_point_cloud_to_file("test_point_cloud.txt")