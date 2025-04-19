#!/usr/bin/python3

import os
import json
from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt

from plotter_3d import *
from forward_kinematics import Twist, ForwardKinematics

class RangeImagePointCloud:
    def __init__(self, fk: ForwardKinematics, fov=(87, 58), range=[17, 5000], downsample=10):
        self.fov = fov
        self.range = range
        self.downsample = downsample
        self.fk = fk

    def point_cloud_from_range_image(self, depth_image, joint_angles):
        body_to_base_tf = self.fk.body_to_base_tf(joint_angles)

        horizontal_degrees_per_pixel = self.fov[0]/depth_image.shape[1]
        vertical_degrees_per_pixel = self.fov[1]/depth_image.shape[0]
        points = []

        for row_i, row in enumerate(depth_image):
            if row_i % self.downsample != 0:
                continue
            for col_i, depth_value in enumerate(row):
                if col_i % self.downsample != 0:
                    continue
                if depth_value < self.range[0] or depth_value > self.range[1]:
                    continue
                horizontal_angle = (-self.fov[0]/2 + col_i * horizontal_degrees_per_pixel) * np.pi/180
                vertical_angle = (-self.fov[1]/2 + row_i * vertical_degrees_per_pixel) * np.pi/180

                z = depth_value / 1000
                y = z*np.tan(vertical_angle)
                x = z*np.tan(horizontal_angle)
                transformed_pt = body_to_base_tf @ np.array([x, y, z, 1])
                points.append([transformed_pt[0], transformed_pt[1], transformed_pt[2]])
        points = np.array(points)

        return points

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Input Directory with Depth Image and Joint States")
    parser.add_argument("-o", "--output", default=None, help="Output file to save resulting point cloud")
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")

    options = parser.parse_args()

    with open(options.config) as f:
        config = json.load(f)

    twists = [Twist(twist) for twist in config["joint_twists"]]
    links = config["mapping_links"]
    for link in links:
        if link["type"] == "camera":
            reference_config = np.array(link["reference_config"])
    fk = ForwardKinematics(twists, reference_config)

    depth_image = np.loadtxt(os.path.join(options.input, "depth.txt"))
    joint_state = np.loadtxt(os.path.join(options.input, "joints.txt"))

    ripc = RangeImagePointCloud(fk)
    points = ripc.point_cloud_from_range_image(depth_image, joint_state)

    plotter = Plotter3D(ViewEnum.ISO, "Test", [-5, 5], [-5, 5], [-5, 5], interactive=False)

    plotter.add_points(points, "Depth Points")
    plotter.update()

    if (options.output != None):
        np.savetxt(options.output, points)

    plt.savefig("test.png")
    plt.show()
