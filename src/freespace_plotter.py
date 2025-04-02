#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

class ViewEnum(Enum):
    ISO = "iso"
    RIGHT = "right"
    FRONT = "front"
    TOP = "top"
    ISO_BACK = "iso_back"
    CUSTOM = "custom"

class FreespacePlotter:
    def __init__(self, view: ViewEnum, title, xlim, ylim, zlim, interactive=True, elev=0, azim=0) -> None:
        if interactive:
            plt.ion()

        self.interactive = interactive
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.axes.set_xlim3d(xlim[0], xlim[1])
        self.ax.axes.set_ylim3d(ylim[0], ylim[1])
        self.ax.axes.set_zlim3d(zlim[0], zlim[1])
        self.ax.axes.set_box_aspect((1, 1, 1))

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        if view == ViewEnum.ISO:
            self.ax.view_init(elev=30, azim=45)
        elif view == ViewEnum.ISO_BACK:
            self.ax.view_init(elev=30, azim=-135)
        elif view == ViewEnum.RIGHT:
            self.ax.view_init(elev=0, azim=0)
        elif view == ViewEnum.FRONT:
            self.ax.view_init(elev=0, azim=-90)
        elif view == ViewEnum.TOP:
            self.ax.view_init(elev=90, azim=-90)
        elif view == ViewEnum.CUSTOM:
            self.ax.view_init(elev=elev, azim=azim)

        plt.title(title)

        self.legend_shown = False

    def update(self):
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def add_points(self, points, legend_str="Free"):
        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c="#1f77b4")

        if not self.legend_shown:
            plt.legend([legend_str])
            self.legend_shown = True

        if self.interactive:
            self.update()

    def cleanup(self):
        plt.close(self.fig)
