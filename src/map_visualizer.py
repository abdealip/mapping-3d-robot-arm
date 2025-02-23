#!/usr/bin/python3

import os
from grid3D import Grid3D
import matplotlib.pyplot as plt
from argparse import ArgumentParser

base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))

parser = ArgumentParser()
parser.add_argument("-m", "--map", type=str, required=True, help="Map File to Visualize")
parser.add_argument("-t", "--title", type=str, help="Plot Title")
parser.add_argument("-v", "--view", type=str, choices=["iso", "right", "front", "top"], default="iso")
args = vars(parser.parse_args())

grid = Grid3D.init_from_file(args["map"])
freespace_points = grid.get_all_points_matching_value(0)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(freespace_points[:, 0], freespace_points[:, 1], freespace_points[:, 2], s=1)

plt.legend(["Free"])
if "title" in args:
    plt.title(args["title"])
else:
    plt.title("Free Space")

ax.axes.set_xlim3d(-0.5, 1.0)
ax.axes.set_ylim3d(-0.5, 1.0)
ax.axes.set_zlim3d(-0.5, 1.0)
ax.axes.set_box_aspect((1, 1, 1))

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

if args["view"] == "iso":
    ax.view_init(elev=30, azim=45)
elif args["view"] == "right":
    ax.view_init(elev=0, azim=0)
elif args["view"] == "front":
    ax.view_init(elev=0, azim=-90)
elif args["view"] == "top":
    ax.view_init(elev=90, azim=-90)

plt.show()
