#!/usr/bin/python3

import os
import json
import numpy as np
import matplotlib.pyplot as plt
from grid3D import BooleanGrid3D
from argparse import ArgumentParser
from plotter_3d import Plotter3D, ViewEnum
from dir_util import make_filename

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Directory containing point clouds")
    parser.add_argument("-m", "--map", required=True, help="camera map file")

    repo_dir = os.path.dirname(os.path.dirname(__file__))
    env_json_file = os.path.join(repo_dir, "config", "environment.json")

    parser.add_argument("-e", "--env", default=env_json_file, help="environment json file")

    options = parser.parse_args()

    with open(options.env) as f:
        env_json = json.load(f)

    with open(options.map) as f:
        grid = BooleanGrid3D.init_from_file(f)

    plotter = Plotter3D(ViewEnum.ISO_BACK, "Map vs Environment", [-1, 1], [-1, 1], [-1, 1])
    plotter.add_points(grid.changed_points)

    legend = ["Mapped points"]

    colors = ['tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
    i = 0
    for mesh in env_json["meshes"]:
        tf_prior = np.array(mesh["transform"])
        point_cloud = np.loadtxt(os.path.join(options.input, make_filename(mesh["description"], "txt")))

        plotter.add_points(point_cloud, color=colors[i])
        legend.append(mesh["description"])

        i = (i + 1) % len(colors)

    plotter.add_legend(legend)

    plt.show()
