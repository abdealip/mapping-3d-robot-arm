#!/usr/bin/python3

import os
import time
import pandas as pd
from argparse import ArgumentParser

from mapper import Mapper, exit_event

class OfflineMapper(Mapper):
    def __init__(self, json_file, joint_file, do_live_display=False, video_dir=None):
        super().__init__(json_file, do_live_display, video_dir=video_dir)

        self.df = pd.read_csv(joint_file)
        self.do_live_display = do_live_display

    def process_joint_log_task(self, profile):
        poses_consumed = 0
        n_rows = len(self.df)
        for i in range(n_rows):
            if exit_event.is_set():
                break
            row = self.df.iloc[i]
            timestamp = row["timestamp"]/1e9
            joint_angles = [row["joint1"], row["joint2"], row["joint3"], row["joint4"], row["joint5"], row["joint6"]]
            if self.consume_joint_angles(joint_angles, timestamp/1e9, profile):
                poses_consumed += 1
                if poses_consumed % 20 == 0:
                    print(f"Consumed {poses_consumed} poses")
                    print(f"Read {i+1} rows out of {n_rows}. {(i+1)/n_rows * 100:.2f}% complete")

        print(f"Consumed {poses_consumed} poses")
        print(f"Read {i+1} rows out of {n_rows}. {(i+1)/n_rows * 100:.2f}% complete")
        if profile:
            print(f"Voxel Size: {self.map.voxel_size}")
            print(f"Average Map Update Runtime: {self.get_average_update_runtime()}")

if __name__ == "__main__":
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser = ArgumentParser()
    parser.add_argument("-m", "--map-file", type=str, required=True, help="Filename to Save Output To")
    parser.add_argument("-j", "--joint-log", type=str, required=True, help="Filename of Joint States Log")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")
    parser.add_argument("--live-display", action="store_true", help="Whether to do a live display of the map updating")
    parser.add_argument("-v", "--video-dir", type=str, default=None, help="directory to store video data")
    parser.add_argument("--profile", action="store_true", help="Whether to do time profiling of updates or not")
    args = vars(parser.parse_args())

    m = OfflineMapper(args["config"], args["joint_log"], do_live_display=args["live_display"], video_dir=args["video_dir"])
    m.process_joint_log_task(args["profile"])
    m.write_to_file(args["map_file"])
    m.cleanup()
