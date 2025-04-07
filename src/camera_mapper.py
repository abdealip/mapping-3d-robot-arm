#!/usr/bin/python3

import os
import json
from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
import rospy

from plotter_3d import *
from camera_joint_snapshot import CameraJointTracker
from forward_kinematics import Twist, ForwardKinematics
from range_image_to_point_cloud import RangeImagePointCloud
from grid3D import BooleanGrid3D

class CameraMapper:
    def __init__(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        twists = [Twist(twist) for twist in config["joint_twists"]]
        links = config["mapping_links"]
        for link in links:
            if link["type"] == "camera":
                reference_config = np.array(link["reference_config"])
                break

        fk = ForwardKinematics(twists, reference_config)
        self.ripc = RangeImagePointCloud(fk)

        self.points = np.empty([0, 3])

        self.x_range = config["x_range"]
        self.y_range = config["y_range"]
        self.z_range = config["z_range"]
        voxel_size = config["voxel_size"]
        self.map = BooleanGrid3D(self.x_range, self.y_range, self.z_range, voxel_size)

    def process_snapshot(self, depth_image, joint_states):
        points = self.ripc.point_cloud_from_range_image(depth_image, joint_states)
        self.points = np.row_stack([self.points, points])
        self.map.mark_start_of_update()
        self.map.set_voxels_from_point_cloud(points)
        return points, self.map.get_changed_points_since_update()

    def process_snapshot_from_dir(self, input_dir):
        depth_image = np.loadtxt(os.path.join(input_dir, "depth.txt"))
        joint_states = np.loadtxt(os.path.join(input_dir, "joints.txt"))
        return self.process_snapshot(depth_image, joint_states)

    def write_point_cloud_to_file(self, filename):
        np.savetxt(filename, self.points)

    def write_map_to_file(self, map_filename):
        with open(map_filename, "w") as f:
            self.map.write_to_file(f)

def rmdir_recursive(dir):
    for name in os.listdir(dir):
        path = os.path.join(dir, name)
        if os.path.isfile(path):
            os.remove(path)
        else:
            rmdir_recursive(path)
    os.rmdir(dir)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", default=None, help="Input Directory with frame subdirs containing depth images and joint states")
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
    config_file_default = os.path.join(base_dir, "config", "map_config.json")
    parser.add_argument("-c", "--config", type=str, default=config_file_default, help=f"JSON Configuration File (default: {config_file_default})")
    parser.add_argument("-o", "--output", default=None, help="Output directory to store frames and map")
    parser.add_argument("--store-raw-points", action="store_true", default=False, help="Whether to store raw point cloud as well as rasterized map")

    options = parser.parse_args()

    cm = CameraMapper(options.config)

    if options.input != None:
        for root, dirs, file in os.walk(options.input):
            if root == options.input:
                for dir in dirs:
                    if "frame" in dir:
                        cm.process_snapshot_from_dir(os.path.join(root, dir))
                break

        plotter = Plotter3D(ViewEnum.ISO_BACK, "Point Cloud", cm.x_range, cm.y_range, cm.z_range, interactive=False, figsize=[7, 7])

        plotter.add_points(np.array(cm.map.changed_points), "Depth Points")
        # plotter.add_points(cm.points, "Depth Points")

        print(f"{len(cm.points)} raw points")
        print(f"{cm.map.get_num_changed_cells()} rasterized points")

        if options.store_raw_points:
            cm.write_point_cloud_to_file(os.path.join(options.input, "point_cloud.txt"))
        plt.savefig(os.path.join(options.input, "point_cloud.png"))
        cm.write_map_to_file(os.path.join(options.input, "camera_map.map"))

        plt.show()

    else:
        if options.output == None:
            print("If not running in offline mode, must specify output directory")
            exit(1)

        if os.path.isdir(options.output):
            res = input(f"Warning: Directory {options.output} exists. Overwrite? [y/n] ")
            while res not in ["y", "n"]:
                res = input("Please type y to overwrite, n to cancel: ")
            if res == "y":
                rmdir_recursive(options.output)
            else:
                exit(0)
        os.mkdir(options.output)

        cjt = CameraJointTracker()
        point_cloud_plotter = Plotter3D(ViewEnum.ISO_BACK, "Point Cloud", cm.x_range, cm.y_range, cm.z_range, figsize=[7, 7])
        fig = plt.figure(constrained_layout=True, figsize=(7, 4))

        frame = 1

        while not rospy.is_shutdown():
            user = input("Press Enter to capture a new frame, exit to exit: ")
            if user == "exit":
                rospy.signal_shutdown("User")
                break

            outpath = os.path.join(options.output, f"frame{frame}")

            os.mkdir(outpath)

            color_image, depth_image, joint_state = cjt.snapshot_with_color()

            fig.add_subplot(2, 1, 1)
            cjt.camera.plot_depth_image(depth_image, fig)
            fig.add_subplot(2, 1, 2)
            cjt.camera.plot_color_image(color_image, fig)

            fig.canvas.draw()
            fig.canvas.flush_events()

            new_points_raw, new_points_rasterized = cm.process_snapshot(depth_image, joint_state)
            print(f"Added {len(new_points_raw)} raw points")
            print(f"Added {len(new_points_rasterized)} rasterized points")
            # point_cloud_plotter.add_points(new_points_rasterized, "Rasterized Depth Points")
            point_cloud_plotter.add_points(new_points_raw, "Depth Points")

            cjt.camera.save_rgb_image(color_image, os.path.join(outpath, "color.png"))
            cjt.camera.save_depth_image(depth_image, os.path.join(outpath, "depth.txt"))
            np.savetxt(os.path.join(outpath, "joints.txt"), joint_state)

            frame += 1

        if options.store_raw_points:
            cm.write_point_cloud_to_file(os.path.join(options.output, "point_cloud.txt"))
        point_cloud_plotter.fig.savefig(os.path.join(options.output, "point_cloud.png"))
        cm.write_map_to_file(os.path.join(options.output, "camera_map.map"))
