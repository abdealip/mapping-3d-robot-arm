#!/usr/bin/python3

import os
import json
import numpy as np
from stl_util import STL
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from plotter_3d import Plotter3D, ViewEnum
from dir_util import clear_and_mkdir_with_confirmation, make_filename

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-o", "--output", default=None, help="Output directory to store transformed point clouds")

    repo_dir = os.path.dirname(os.path.dirname(__file__))
    env_json_file = os.path.join(repo_dir, "config", "environment.json")

    parser.add_argument("-e", "--env", default=env_json_file, help="environment json file")
    parser.add_argument("-t", "--title", default="Environment from CAD Models", help="Plot Title")

    options = parser.parse_args()

    if options.output != None:
        clear_and_mkdir_with_confirmation(options.output)

    with open(options.env) as f:
        env = json.load(f)

    points = []

    for model in env["meshes"]:
        stl_path = os.path.join(repo_dir, "models", model["model"])
        mesh = STL(stl_path)
        pts = mesh.get_rasterized_points(env["point_density"])
        transform = np.array(model["transform"])
        pts = np.hstack([pts, np.ones([pts.shape[0], 1])])
        pts = (transform @ pts.T).T
        points.append(pts)

        if options.output != None:
            outf = os.path.join(options.output, make_filename(model["description"], "txt"))
            np.savetxt(outf, pts[:, :3])

    points = np.array(points).reshape([-1, 4])

    plotter = Plotter3D(ViewEnum.ISO_BACK, options.title, [-1, 1], [-1, 1], [-1, 1], interactive=False)
    plotter.add_points(points[:, :3])

    plt.show()
