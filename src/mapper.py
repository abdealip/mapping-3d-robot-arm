import os
import json
import time
import signal
import threading
import numpy as np
from typing import List

from grid3D import BooleanGrid3D
from forward_kinematics import Twist, TransformableCylinder, joint_angles_close
from plotter_3d import ViewEnum, Plotter3D

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()

class Mapper:
    def __init__(self, json_file, do_live_display=False, video_dir=None):
        with open(json_file) as f:
            self.config = json.load(f)

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


        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        x_range = self.config["x_range"]
        y_range = self.config["y_range"]
        z_range = self.config["z_range"]
        voxel_size = self.config["voxel_size"]
        self.map = BooleanGrid3D(x_range, y_range, z_range, voxel_size)

        self.do_live_display = do_live_display
        if self.do_live_display:
            self.plotter = Plotter3D(ViewEnum.ISO, "Map", [self.map.xmin, self.map.xmax],
                                            [self.map.ymin, self.map.ymax], [self.map.zmin, self.map.zmax])
            self.plotter.update()
            self.video_dir = None
            self.frame_idx=0
            if video_dir != None:
                self.video_dir = video_dir
                if os.path.isdir(video_dir):
                    os.rmdir(video_dir)
                os.mkdir(video_dir)
                self.img_idx_file = open(os.path.join(video_dir, "img_index.csv"), "w")
                self.img_idx_file.write("imgfile,timestamp\n")

        self.total_update_runtime = 0   # total update runtime for this session
        self.num_updates = 0            # number of pose updates that have happened in this session
        self.num_poses = 0              # total number of poses used to build this map

        self.min_joint_movement_radians = self.config["min_joint_movement_degrees"] * np.pi/180
        self.sample_time_seconds = self.config["sample_time_seconds"]

        self.last_joint_angles = None
        self.last_timestamp = None

    def read_from_file(self, map_filename):
        with open(map_filename, "r") as f:
            self.map = BooleanGrid3D.init_from_file(f)
            num_poses_line = f.readline().split()
            self.num_poses = int(num_poses_line[1])

    def write_to_file(self, map_filename):
        with open(map_filename, "w") as f:
            self.map.write_to_file(f)
            f.write(f"num_updates: {self.num_updates}")

    def consume_joint_angles(self, joint_angles, timestamp, profile=False):
        if len(joint_angles) != len(self.twists):
            print("ERROR: differing number of joint angles and joint twists")
            return False

        if self.last_joint_angles != None and joint_angles_close(self.last_joint_angles, joint_angles, self.min_joint_movement_radians):
            return False

        if self.last_timestamp != None and timestamp - self.last_timestamp < self.sample_time_seconds:
            return False

        self.last_joint_angles = joint_angles
        self.last_timestamp = timestamp

        if self.do_live_display:
            self.map.mark_start_of_update()

        if profile:
            start_time = time.time()

        for cylinder in self.cylinders:
            cylinder.transform(joint_angles)
            self.map.update_points_within_cylinder(cylinder)

        if profile:
            self.total_update_runtime += time.time() - start_time

        if self.do_live_display:
            new_points = self.map.get_changed_points_since_update()
            if len(new_points) > 0:
                self.plotter.add_points(new_points)
                if self.video_dir != None:
                    fname = f"frame{self.frame_idx:04d}.png"
                    self.plotter.fig.savefig(os.path.join(self.video_dir, fname))
                    self.img_idx_file.write(f"{fname},{timestamp}\n")
                    self.img_idx_file.flush()
                    self.frame_idx += 1

        self.num_updates += 1
        self.num_poses += 1
        return True

    def cleanup(self):
        if self.do_live_display and self.video_dir != None:
            self.img_idx_file.close()

    def get_average_update_runtime(self):
        if self.num_updates == 0:
            return None
        else:
            return self.total_update_runtime / self.num_updates

    def get_num_poses_consumed(self):
        return self.num_poses

    def get_num_free_cells(self):
        return self.map.get_num_free_cells()
