#!/usr/bin/python3

import os
import json
from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt

from freespace_plotter import *
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

        fk = ForwardKinematics(twists, reference_config)
        self.ripc = RangeImagePointCloud(fk)

        self.points = np.empty([0, 3])

    def process_snapshot(self, input_dir):
        depth_image = np.loadtxt(os.path.join(input_dir, "depth.txt"))
        joint_state = np.loadtxt(os.path.join(input_dir, "joints.txt"))

        points = self.ripc.point_cloud_from_range_image(depth_image, joint_state)
        self.points = np.row_stack([self.points, points])

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Input Directory with Depth Image and Joint States")
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")

    options = parser.parse_args()

    cm = CameraMapper(options.config)
    cm.process_snapshot(options.input)

    plotter = FreespacePlotter(ViewEnum.ISO, "Test", [-5, 5], [-5, 5], [-5, 5], interactive=False)

    plotter.add_points(cm.points, "Depth Points")
    plotter.update()

    print(len(cm.points))

    plt.savefig("test.png")
    plt.show()
