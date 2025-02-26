#!/usr/bin/python3

import os
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser

from grid3D import BooleanGrid3D
from freespace_plotter import FreespacePlotter, ViewEnum

if __name__ == "__main__":
    base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))

    parser = ArgumentParser()
    parser.add_argument("-m", "--map", type=str, required=True, help="Map File to Visualize")
    parser.add_argument("-t", "--title", type=str, default="Map", help="Plot Title")
    parser.add_argument("-v", "--view", type=str, choices=[e.value for e in ViewEnum], default=ViewEnum.ISO.value)
    args = vars(parser.parse_args())

    with open(args["map"]) as f:
        grid = BooleanGrid3D.init_from_file(f)
    freespace_points = np.array(grid.changed_points)

    plotter = FreespacePlotter(ViewEnum(args["view"]), args["title"], [grid.xmin, grid.xmax],
                               [grid.ymin, grid.ymax], [grid.zmin, grid.zmax], interactive=False)
    plotter.add_points(freespace_points)
    plt.show()
