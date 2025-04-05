#!/usr/bin/python3

import os
import json
import numpy as np
from argparse import ArgumentParser

from grid3D import BooleanGrid3D

class PointCloudRasterizer:
    def __init__(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        self.x_range = config["x_range"]
        self.y_range = config["y_range"]
        self.z_range = config["z_range"]
        voxel_size = config["voxel_size"]
        self.map = BooleanGrid3D(self.x_range, self.y_range, self.z_range, voxel_size)

    def write_map_to_file(self, map_filename):
        with open(map_filename, "w") as f:
            self.map.write_to_file(f)

    def process_point_cloud_file(self, point_cloud_file):
        raw_points = np.loadtxt(point_cloud_file)
        self.map.set_voxels_from_point_cloud(raw_points)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Raw point cloud text file")
    parser.add_argument("-o", "--output", required=True, help="Output map file")
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")

    options = parser.parse_args()

    rasterizer = PointCloudRasterizer(options.config)
    rasterizer.process_point_cloud_file(options.input)
    rasterizer.write_map_to_file(options.output)
