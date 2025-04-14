#!/usr/bin/python3

import os
import json
import numpy as np
import matplotlib.pyplot as plt
from plotter_3d import Plotter3D, ViewEnum
from stl_util import STL

repo_path = os.path.dirname(os.path.dirname(__file__))

filepath = os.path.join(repo_path, "config", "environment.json")
with open(filepath) as f:
    env = json.load(f)

points = []

for model in env["meshes"]:
    stl_path = os.path.join(repo_path, "models", model["model"])
    mesh = STL(stl_path)
    pts = mesh.get_rasterized_points(env["point_density"])
    transform = np.array(model["transform"])
    pts = np.hstack([pts, np.ones([pts.shape[0], 1])])
    pts = (transform @ pts.T).T
    points.append(pts)

points = np.array(points).reshape([-1, 4])

plotter = Plotter3D(ViewEnum.ISO_BACK, "environment from CAD model", [-1, 1], [-1, 1], [-1, 1], interactive=False)
plotter.add_points(points[:, :3], "CAD models as point clouds")

plt.show()
