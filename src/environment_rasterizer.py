import os
import json
import numpy as np
import matplotlib.pyplot as plt
from plotter_3d import Plotter3D, ViewEnum

class Polygon:
    def __init__(self, points_2d):
        self.points_2d = np.array(points_2d)
        self.xmin = np.min(self.points_2d[:, 0])
        self.xmax = np.max(self.points_2d[:, 0])
        self.ymin = np.min(self.points_2d[:, 1])
        self.ymax = np.max(self.points_2d[:, 1])

    def contains_point(self, point_2d):
        x, y = point_2d
        n = self.points_2d.shape[0]
        inside = False

        for i in range(n):
            xi, yi = self.points_2d[i, :]
            xj, yj = self.points_2d[(i + 1) % n, :]

            # Check if point is within the y-bounds of the edge
            if (yi > y) != (yj > y):
                # Compute x coordinate of the intersection of the edge with the horizontal ray
                x_intersect = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
                if x < x_intersect:
                    inside = not inside

        return inside

    def get_rasterized_points(self, dist_between_points):
        points = []
        x = self.xmin
        while x < self.xmax:
            y = self.ymin
            while y < self.ymax:
                if self.contains_point([x, y]):
                    points.append([x, y])
                y += dist_between_points
            x += dist_between_points
        return np.array(points)

    def remove_contained_points(self, points):
        not_contained_points = []
        for point in points:
            if not self.contains_point(point):
                not_contained_points.append(point)
        return np.array(not_contained_points)

filepath = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "environment.json")
with open(filepath) as f:
    env = json.load(f)

cnc1 = env["polygons"][1]

outer = Polygon(cnc1["outer_points"])
pts = outer.get_rasterized_points(env["parameters"]["point_density"])

inner = Polygon(cnc1["inner_points"])
pts = inner.remove_contained_points(pts)

transform = np.array(cnc1["transform"])

pts = np.hstack([pts, np.zeros([pts.shape[0], 1]), np.ones([pts.shape[0], 1])])
pts = (transform @ pts.T).T

plotter = Plotter3D(ViewEnum.ISO_BACK, "polygon", [-2, 2], [-2, 2], [-2, 2], interactive=False)
plotter.add_points(pts[:, :3], cnc1["description"])

# plt.scatter(pts[:, 0], pts[:, 1], s=1)
plt.show()
