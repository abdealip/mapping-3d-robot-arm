#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

class FreespacePlotter:
    def __init__(self, view, title, xlim, ylim, zlim) -> None:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.axes.set_xlim3d(xlim[0], xlim[1])
        ax.axes.set_ylim3d(ylim[0], ylim[1])
        ax.axes.set_zlim3d(zlim[0], zlim[1])
        ax.axes.set_box_aspect((1, 1, 1))

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        if view == "iso":
            ax.view_init(elev=30, azim=45)
        elif view == "right":
            ax.view_init(elev=0, azim=0)
        elif view == "front":
            ax.view_init(elev=0, azim=-90)
        elif view == "top":
            ax.view_init(elev=90, azim=-90)

        self.points = np.zeros([0, 3])

        plt.title(title)
        plt.legend(["Free"])

    def add_points(self, points):
        self.points = np.append(self.points, points)
        # display.update() ?
