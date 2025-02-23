import numpy as np

class Cylinder:
    def __init__(self, radius, length, base_point=[0, 0, 0], direction=[0, 0, 1]):
        self.radius = radius
        self.length = length
        self.base_point = np.array(base_point)
        self.direction = np.array(direction)/np.linalg.norm(direction)    # unit vector
    
    def set_base_point(self, base_point):
        self.base_point = base_point
    
    def set_direction(self, direction):
        self.direction = direction / np.linalg.norm(direction)

    def get_bounds_x(self):
        top_point = self.base_point + self.length * self.direction
        proj_length_yz_plane = np.linalg.norm(top_point[1:] - self.base_point[1:])
        angle_with_yz_plane = np.arctan((top_point[0] - self.base_point[0])/proj_length_yz_plane)
        xmin = min(self.base_point[0] - self.radius*np.cos(angle_with_yz_plane), top_point[0] - self.radius*np.cos(angle_with_yz_plane))
        xmax = max(top_point[0] + self.radius*np.cos(angle_with_yz_plane), self.base_point[0] + self.radius*np.cos(angle_with_yz_plane))
        return [xmin, xmax]

    def get_bounds_y(self):
        top_point = self.base_point + self.length * self.direction
        proj_length_xz_plane = np.linalg.norm(top_point[0::2] - self.base_point[0::2])
        angle_with_xz_plane = np.arctan((top_point[1] - self.base_point[1])/proj_length_xz_plane)
        ymin = min(self.base_point[1] - self.radius*np.cos(angle_with_xz_plane), top_point[1] - self.radius*np.cos(angle_with_xz_plane))
        ymax = max(top_point[1] + self.radius*np.cos(angle_with_xz_plane), self.base_point[1] + self.radius*np.cos(angle_with_xz_plane))
        return [ymin, ymax]

    def get_bounds_z(self):
        top_point = self.base_point + self.length * self.direction
        proj_length_xy_plane = np.linalg.norm(top_point[:2] - self.base_point[:2])
        angle_with_xy_plane = np.arctan((top_point[2] - self.base_point[2])/proj_length_xy_plane)
        zmin = min(self.base_point[2] - self.radius*np.cos(angle_with_xy_plane), top_point[2] - self.radius*np.cos(angle_with_xy_plane))
        zmax = max(top_point[2] + self.radius*np.cos(angle_with_xy_plane), self.base_point[2] + self.radius*np.cos(angle_with_xy_plane))
        return [zmin, zmax]

    def get_bounding_box(self):
        xbound = self.get_bounds_x()
        ybound = self.get_bounds_y()
        zbound = self.get_bounds_z()
        return [xbound[0], ybound[0], zbound[0], xbound[1], ybound[1], zbound[1]]

    def contains_point(self, point):
        # step 1: project point onto line defined by cylinder's axis
        p = np.array(point)
        vector_to_project = p - self.base_point
        projected_point = self.base_point + vector_to_project.dot(self.direction) * self.direction

        # # condition 1: if distance from point to projected point is > radius, point is not contained
        if np.linalg.norm(p - projected_point) > self.radius:
            return False

        # condition 2: if projected point is between base_point and top_point, point is contained
        top_point = self.base_point + self.length * self.direction
        for dim in range(3):
            if self.direction[dim] != 0:
                if (projected_point[dim] - self.base_point[dim])/self.direction[dim] > 0 and (projected_point[dim] - top_point[dim])/self.direction[dim] < 0:
                    return True
                else:
                    return False
        
        # if we have reached this point then the direction is [0, 0, 0] so always return False
        print("ERROR: direction vector is zero")
        return True
