#!/usr/bin/python3

from __future__ import annotations
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
        self.initialize_aux()

    def initialize_aux(self):
        self.all_points = self.mesh.vectors.reshape([-1, 3])
        self.calculate_bounds()
        self.make_polygon_array()

    def calculate_bounds(self):
        self.xmin = np.min(self.all_points[:, 0])
        self.xmax = np.max(self.all_points[:, 0])
        self.ymin = np.min(self.all_points[:, 1])
        self.ymax = np.max(self.all_points[:, 1])
        self.zmin = np.min(self.all_points[:, 2])
        self.zmax = np.max(self.all_points[:, 2])

    def make_polygon_array(self):
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

    def transform(self, transform_matrix: np.ndarray):
        '''
        transform_matrix: SE(3) transform to apply to self
        '''

        if transform_matrix.shape != (4, 4):
            raise ValueError("Transform matrix must be a 4x4 SE(3) matrix.")

        # === Transform Vertices ===
        num_triangles = self.mesh.vectors.shape[0]
        points_homogeneous = np.hstack((self.all_points, np.ones([self.all_points.shape[0], 1])))
        transformed_points = (transform_matrix @ points_homogeneous.T).T[:, :3]
        self.mesh.vectors = transformed_points.reshape(num_triangles, 3, 3)

        # === Transform Normals ===
        normals = self.mesh.normals
        zeros = np.zeros((normals.shape[0], 1))
        normals_homogeneous = np.hstack((normals, zeros))
        transformed_normals = (transform_matrix @ normals_homogeneous.T).T[:, :3]

        # Normalize the resulting normals
        norms = np.linalg.norm(transformed_normals, axis=1, keepdims=True)
        safe_normals = np.where(norms > 0, transformed_normals / norms, transformed_normals)

        self.mesh.normals = safe_normals

    def append_other_stl_data(self, other: STL):
        self.mesh.data = np.concatenate([self.mesh.data, other.mesh.data])
        self.initialize_aux()

    def write(self, output_stl_path):
        self.mesh.save(output_stl_path)

if __name__ == "__main__":
    file = "models/cnc_front.stl"

    s = STL(file)
    points = s.get_rasterized_points(0.02)
    print(len(points))

    p = Plotter3D(ViewEnum.ISO_BACK, "cnc_front_untransformed", [-0.1, 0.8], [-0.1, 0.8], [-0.45, 0.45])

    p.add_points(points, "stl")

    plt.show()

