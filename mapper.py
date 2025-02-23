import numpy as np
import json
from grid3D import Grid3D
from cylinder import Cylinder

class Mapper:
    def __init__(self, json_file):
        with open(json_file) as f:
            self.config = json.load(f)
        x_range = self.config["x_range"]
        y_range = self.config["y_range"]
        z_range = self.config["z_range"]
        voxel_size = self.config["voxel_size"]
        self.map = Grid3D(x_range, y_range, z_range, voxel_size, 1)

        # TODO - initialize mapping cylinders from config
        # TODO - initialize joint twists from config, make forward kinematic functions

    def read_from_file(self, map_filename):
        self.map = Grid3D.init_from_file(map_filename)

    def write_to_file(self, map_filename):
        self.map.write_to_file(map_filename)

    def consume_joint_pose(self, pose):
        #TODO - use pose to update base point and direction of cylinders
        #TODO - update based on all cylinders
        cyl = Cylinder(0.2, 3, base_point=[0, 0, 0], direction = [1, 1, 1])
        self.map.set_all_points_within_cylinder_to_value(0, cyl)

if __name__ == "__main__":
    m = Mapper("map_config.json")
    m.consume_joint_pose(5)
    m.write_to_file("test.map")
