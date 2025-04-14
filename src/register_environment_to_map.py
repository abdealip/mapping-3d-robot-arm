#!/usr/bin/python3

import os
import json
import numpy as np
from grid3D import BooleanGrid3D
from argparse import ArgumentParser
from dir_util import make_filename, clear_and_mkdir_with_confirmation
from icp import icp

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Directory containing point clouds")
    parser.add_argument("-m", "--map", required=True, help="camera map file")
    parser.add_argument("-o", "--output", required=True, help="Output json file to write resulting transforms")
    parser.add_argument("-p", "--output-pc", required=True, help="Output directory to write corrected point clouds")

    options = parser.parse_args()

    clear_and_mkdir_with_confirmation(options.output_pc)

    repo_dir = os.path.dirname(os.path.dirname(__file__))
    env_json_file = os.path.join(repo_dir, "config", "environment.json")
    with open(env_json_file) as f:
        env_json = json.load(f)

    with open(options.map) as f:
        grid = BooleanGrid3D.init_from_file(f)

    map_points = np.array(grid.changed_points)

    new_env_json = {}
    new_env_json["point_density"] = env_json["point_density"]
    new_env_json["meshes"] = []

    for mesh in env_json["meshes"]:
        tf_prior = np.array(mesh["transform"])
        point_cloud = np.loadtxt(os.path.join(options.input, make_filename(mesh["description"], "txt")))

        aligned_pts, extra_tf = icp(np.array(grid.changed_points), point_cloud)

        new_tf = extra_tf @ tf_prior

        new_mesh = {
            "description": mesh["description"],
            "model": mesh["model"],
            "transform": new_tf.tolist()
        }
        new_env_json["meshes"].append(new_mesh)

        np.savetxt(os.path.join(options.output_pc, make_filename(mesh["description"], "txt")), aligned_pts)

    with open(options.output, "w") as f:
        json.dump(new_env_json, f, indent=4)
