import json
import time
import signal
import threading
from typing import List
from queue import Queue

from grid3D import BooleanGrid3D
from forward_kinematics import Twist, TransformableCylinder

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()

class Mapper:
    def __init__(self, json_file, do_live_display=False):
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
        self.visualization_queue = None
        if do_live_display:
            self.visualization_queue = Queue()
            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)
            threading.Thread(target=self.visualization_thread)

        x_range = self.config["x_range"]
        y_range = self.config["y_range"]
        z_range = self.config["z_range"]
        voxel_size = self.config["voxel_size"]
        self.map = BooleanGrid3D(x_range, y_range, z_range, voxel_size, self.visualization_queue)

    def read_from_file(self, map_filename):
        self.map = BooleanGrid3D.init_from_file(map_filename, self.visualization_queue)

    def write_to_file(self, map_filename):
        self.map.write_to_file(map_filename)

    def consume_joint_angles(self, joint_angles, profile=False):
        if len(joint_angles) != len(self.twists):
            print("ERROR: differing number of joint angles and joint twists")
            return
        if profile:
            start_time = time.time()
        for cylinder in self.cylinders:
            cylinder.transform(joint_angles)
            self.map.set_all_points_within_cylinder_to_value(0, cylinder)
        if profile:
            timediff = time.time() - start_time
            print(f"Map update took {timediff:.2f} seconds")
            return timediff

    def visualization_thread(self):
        while not exit_event.is_set():
            new_free_points = self.visualization_queue.get(block=True)
            pass
