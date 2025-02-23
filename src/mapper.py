import os
import json
import time
from typing import List
import numpy as np
import pandas as pd

from grid3D import Grid3D
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

        links = self.config["mapping_links"]
        self.cylinders: List[TransformableCylinder] = []
        for link in links:
            if link["type"] == "cylinder":
                n_twists = link["twists"]
                self.cylinders.append(TransformableCylinder(link["radius"],
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

    def consume_joint_log(self, joint_filename, sample_time_seconds):
        df = pd.read_csv(joint_filename)
        last_timestamp = 0
        sample_time_nanoseconds = sample_time_seconds * 1e9
        poses_consumed = 0
        n_rows = len(df)
        for i in range(n_rows):
            row = df.iloc[i]
            timestamp = row["timestamp"]
            if timestamp - last_timestamp > sample_time_nanoseconds:
                joint_angles = [row["joint1"], row["joint2"], row["joint3"], row["joint4"], row["joint5"], row["joint6"]]
                self.consume_joint_angles(joint_angles)
                poses_consumed += 1
                if poses_consumed % 20 == 0:
                    print(f"Consumed {poses_consumed} poses")
                    print(f"Read {i+1} rows out of {n_rows}. {i/n_rows * 100:.2f}% complete")
                last_timestamp = timestamp
        print(f"Consumed {poses_consumed} poses")
        print(f"Read {i+1} rows out of {n_rows}. {(i/n_rows * 100):.2f}% complete")

if __name__ == "__main__":
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file = os.path.join(base_dir, "config", "map_config.json")
    map_file = os.path.join(base_dir, "data", "test.map")
    joint_log = os.path.join(base_dir, "data", "joint_states.csv")
    m = Mapper(config_file)
    m.consume_joint_log(joint_log, 0.1)
    # m.consume_joint_angles([np.pi/4, np.pi/6, np.pi/2, 0, 0, 0])
    m.write_to_file(map_file)
