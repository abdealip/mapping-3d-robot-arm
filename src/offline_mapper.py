#!/usr/bin/python3

import os
import pandas as pd
from argparse import ArgumentParser

from mapper import Mapper

class OfflineMapper(Mapper):
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
                    print(f"Read {i+1} rows out of {n_rows}. {(i+1)/n_rows * 100:.2f}% complete")
                last_timestamp = timestamp
        print(f"Consumed {poses_consumed} poses")
        print(f"Read {i+1} rows out of {n_rows}. {(i+1)/n_rows * 100:.2f}% complete")

if __name__ == "__main__":
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser = ArgumentParser()
    parser.add_argument("-m", "--map-file", type=str, required=True, help="Filename to Save Output To")
    parser.add_argument("-j", "--joint-log", type=str, required=True, help="Filename of Joint States Log")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")
    parser.add_argument("-s", "--sample-time", type=float, default=0.1, help="Sample Time of Joint States in Seconds")
    parser.add_argument("--live-display", action="store_true", help="Whether to do a live display of the map updating")
    args = vars(parser.parse_args())

    m = OfflineMapper(args["config"], do_live_display=args["live_display"])
    m.consume_joint_log(args["joint_log"], args["sample_time"])
    m.write_to_file(args["map_file"])
