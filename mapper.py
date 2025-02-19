import numpy as np
import json
from grid3D import Grid3D

class Mapper:
    def __init__(self, json_file):
        with open(json_file) as f:
            self.config = json.load(f)
        x_range = self.config["x_range"]
        y_range = self.config["y_range"]
        z_range = self.config["z_range"]
        voxel_size = self.config["voxel_size"]
        self.map = Grid3D(x_range, y_range, z_range, voxel_size, 1)

    def read_from_file(self, map_filename):
        pass

    def write_to_file(self, map_filename):
        self.map.write_to_file(map_filename)

    def consume_joint_pose(self, pose):
        if len(pose) != len(self.link_lengths) + 1:
            print("ERROR: Joint pose dimension inconsistent with link lengths")
            return

        # pose is a list of joint angles
        # need to get base frame coordinates of each joint using forward kinematics

if __name__ == "__main__":
    m = Mapper("map_config.json")
    m.write_to_file("test.map")
    
