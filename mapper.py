from typing import List
import numpy as np
import json
from grid3D import Grid3D
from cylinder import Cylinder
from forward_kinematics import Twist, TransformableCylinder

class Mapper:
    def __init__(self, json_file):
        with open(json_file) as f:
            self.config = json.load(f)
        x_range = self.config["x_range"]
        y_range = self.config["y_range"]
        z_range = self.config["z_range"]
        voxel_size = self.config["voxel_size"]
        self.map = Grid3D(x_range, y_range, z_range, voxel_size, 1)

        twists = self.config["joint_twists"]
        self.twists = [Twist(twist) for twist in twists]

        links = self.config["links"]
        self.cylinders: List[TransformableCylinder] = []
        for link in links:
            if link["type"] == "cylinder":
                n_twists = link["twists"]
                self.cylinders.append(TransformableCylinder(link["mapping_radius"],
                                                            link["length"],
                                                            link["axis"],
                                                            link["reference_config"],
                                                            self.twists[:n_twists]))

    def read_from_file(self, map_filename):
        self.map = Grid3D.init_from_file(map_filename)

    def write_to_file(self, map_filename):
        self.map.write_to_file(map_filename)

    def consume_joint_angles(self, joint_angles):
        if len(joint_angles) != len(self.twists):
            print("ERROR: differing number of joint angles and joint twists")
            return
        for cylinder in self.cylinders:
            cylinder.transform(joint_angles)
            self.map.set_all_points_within_cylinder_to_value(0, cylinder)

if __name__ == "__main__":
    m = Mapper("map_config.json")
    m.consume_joint_angles([np.pi/4, np.pi/6, -np.pi/2, 0, 0, 0])
    m.write_to_file("test.map")
