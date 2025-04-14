import numpy as np

def is_point_on_segment(px, py, x1, y1, x2, y2, eps=1e-9):
    # Check if point (px, py) is on segment [(x1, y1), (x2, y2)]
    cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
    if abs(cross) > eps:
        return False  # not colinear

    dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
    return dot <= eps  # between the endpoints

class Polygon2D:
    def __init__(self, points_2d):
        self.points_2d = np.array(points_2d)
        self.xmin = np.min(self.points_2d[:, 0])
        self.xmax = np.max(self.points_2d[:, 0])
        self.ymin = np.min(self.points_2d[:, 1])
        self.ymax = np.max(self.points_2d[:, 1])

    def contains_point(self, point_2d, eps=1e-9):
        x, y = point_2d
        n = self.points_2d.shape[0]
        inside = False

        for i in range(n):
            xi, yi = self.points_2d[i, :]
            xj, yj = self.points_2d[(i + 1) % n, :]

            # Check if point is on the edge
            if is_point_on_segment(x, y, xi, yi, xj, yj, eps):
                return True

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


class Polygon3D:
    def __init__(self, points_3d):
        if len(points_3d) < 3:
            raise ValueError("Polygon must have at least 3 vertices.")
        self.points = np.array(points_3d)
        self.calculate_bounds()

    def calculate_bounds(self):
        self.xmin = np.min(self.points[:, 0])
        self.xmax = np.max(self.points[:, 0])
        self.ymin = np.min(self.points[:, 1])
        self.ymax = np.max(self.points[:, 1])
        self.zmin = np.min(self.points[:, 2])
        self.zmax = np.max(self.points[:, 2])

    def contains_point(self, point, tolerance):
        """
        Determines if a 3D point is inside a 3D polygon (assumed planar), within a given tolerance from the polygon's plane.

        Args:
            point (array-like): The 3D query point.
            tolerance (float): Maximum allowed distance from the polygon plane.

        Returns:
            bool: True if the point lies within the polygon (within tolerance), False otherwise.
        """

        if point[0] < self.xmin - tolerance or point[0] > self.xmax + tolerance or  \
           point[1] < self.ymin - tolerance or point[1] > self.ymax + tolerance or  \
           point[2] < self.zmin - tolerance or point[2] > self.zmax + tolerance:
            return False

        point = np.array(point)

        # Define plane using the first three points
        p0, p1, p2 = self.points[0, :], self.points[1, :], self.points[2, :]
        normal = np.cross(p1 - p0, p2 - p0)
        normal = normal / np.linalg.norm(normal)

        # Compute distance from point to plane
        vec_to_point = point - p0
        distance_to_plane = np.dot(vec_to_point, normal)

        if abs(distance_to_plane) > tolerance:
            return False  # too far from the plane

        # Project the point onto the plane
        projected_point = point - distance_to_plane * normal

        # Create local 2D coordinate system in the polygon's plane
        x_axis = (p1 - p0)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis)

        def project_onto_plane_2d(p):
            v = p - p0
            return [np.dot(v, x_axis), np.dot(v, y_axis)]

        point_2d = project_onto_plane_2d(projected_point)
        polygon_2d = Polygon2D([project_onto_plane_2d(p) for p in self.points])

        return polygon_2d.contains_point(point_2d)

    def get_rasterized_points(self, dist_between_points):
        points = []
        x = self.xmin
        while x <= self.xmax:
            y = self.ymin
            while y <= self.ymax:
                z = self.zmin
                while z <= self.zmax:
                    if self.contains_point([x, y, z], dist_between_points/2):
                        points.append([x, y, z])
                    z += dist_between_points
                y += dist_between_points
            x += dist_between_points
        return np.array(points)

    def transform(self, tf: np.ndarray):
        '''
        tf: SE(3) transform to apply to self
        '''
        points_homog = np.hstack([self.points, np.ones([len(self.points), 1])])
        points_homog_new = (tf @ points_homog.T).T
        self.points = points_homog_new[:, :3]
        self.calculate_bounds()
