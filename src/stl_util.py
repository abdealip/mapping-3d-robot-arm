#!/usr/bin/python3

from typing import List
from stl import mesh
import numpy as np
from polygon_util import Polygon3D
from plotter_3d import Plotter3D, ViewEnum
import matplotlib.pyplot as plt

class STL:
    def __init__(self, stl_filename):
        self.mesh = mesh.Mesh.from_file(stl_filename)
        self.mesh.vectors *= 0.001                      # STL units are in mm
        all_points = self.mesh.vectors.reshape([-1, 3])
        self.xmin = np.min(all_points[:, 0])
        self.xmax = np.max(all_points[:, 0])
        self.ymin = np.min(all_points[:, 1])
        self.ymax = np.max(all_points[:, 1])
        self.zmin = np.min(all_points[:, 2])
        self.zmax = np.max(all_points[:, 2])

        self.triangles: List[Polygon3D] = []
        for i in range(len(self.mesh.vectors)):
            self.triangles.append(Polygon3D(self.mesh.vectors[i]))

    def get_rasterized_points(self, dist_between_points):
        points = []
        x = self.xmin
        while x <= self.xmax + dist_between_points/2:
            y = self.ymin
            while y <= self.ymax + dist_between_points/2:
                z = self.zmin
                while z <= self.zmax + dist_between_points/2:
                    for triangle in self.triangles:
                        if triangle.contains_point([x, y, z], dist_between_points/2):
                            points.append([x, y, z])
                            break       # no need to check other triangles now
                    z += dist_between_points
                y += dist_between_points
            x += dist_between_points
        return np.array(points)

if __name__ == "__main__":
    file = "models/cnc_front.stl"

    s = STL(file)
    points = s.get_rasterized_points(0.02)
    print(len(points))

    p = Plotter3D(ViewEnum.ISO_BACK, "cnc_front_untransformed", [-0.1, 0.8], [-0.1, 0.8], [-0.45, 0.45])

    p.add_points(points, "stl")

    plt.show()

