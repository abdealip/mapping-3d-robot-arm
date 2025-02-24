import numpy as np
from queue import Queue

from cylinder import Cylinder

class Grid3D:
    def __init__(self, x_range, y_range, z_range, voxel_size, initial_value: int):
        self.xmin = x_range[0]
        self.xmax = x_range[1]
        self.ymin = y_range[0]
        self.ymax = y_range[1]
        self.zmin = z_range[0]
        self.zmax = z_range[1]
        self.voxel_size = voxel_size
    
        self.xdim = int((self.xmax - self.xmin) // self.voxel_size)
        self.ydim = int((self.ymax - self.ymin) // self.voxel_size)
        self.zdim = int((self.zmax - self.zmin) // self.voxel_size)
        self.voxels = np.full([self.xdim, self.ydim, self.zdim], initial_value, dtype=int)

    def write_to_file(self, outf):
        lines = []
        lines.append(f"range: {self.xmin} {self.xmax} {self.ymin} {self.ymax} {self.zmin} {self.zmax}\n")
        lines.append(f"resolution: {self.voxel_size}\n")
        lines.append(f"shape: {self.xdim} {self.ydim} {self.zdim}\n")
        data = self.voxels.flatten()
        data_str = " ".join([str(datum) for datum in data])
        lines.append(f"data: {data_str}\n")
        outf.writelines(lines)

    @classmethod
    def init_from_file(cls, f):
        grid = None
        
        rangeline = f.readline().split(' ')
        resolutionline = f.readline().split(' ')
        shapeline = f.readline().split(' ')
        dataline = f.readline().split(' ')
        
        x_range = [float(rangeline[1]), float(rangeline[2])]
        y_range = [float(rangeline[3]), float(rangeline[4])]
        z_range = [float(rangeline[5]), float(rangeline[6])]
        voxel_size = float(resolutionline[1])
        shape = [int(shapeline[1]), int(shapeline[2]), int(shapeline[3])]

        grid = cls(x_range, y_range, z_range, voxel_size, 0)
        data = np.array([int(datum) for datum in dataline[1:]])
        grid.voxels = data.reshape(shape)
        return grid

    def in_range(self, x, y, z) -> bool:
        return (x > self.xmin and x < self.xmax and y > self.ymin and y < self.ymax and z > self.zmin and z < self.zmax)

    def idx_in_range(self, x_i, y_i, z_i) -> bool:
        return (x_i >= 0 and x_i < self.xdim and y_i >= 0 and y_i < self.ydim and z_i >= 0 and z_i < self.zdim)

    def set_voxel_at(self, x, y, z, value):
        if self.in_range(x, y, z):
            idxs = self.cartesian_to_idxs(x, y, z)
            self.voxels[idxs[0], idxs[1], idxs[2]] = value
        else:
            print("ERROR: Out of range (set_voxel_at)")

    def get_voxel_at(self, x, y, z):
        if self.in_range(x, y, z):
            idxs = self.cartesian_to_idxs(x, y, z)
            return self.voxels[idxs[0], idxs[1], idxs[2]]
        else:
            print("ERROR: Out of range (get_voxel_at)")
            return None

    def set_voxel_at_idx(self, x_i, y_i, z_i, value):
        if self.idx_in_range(x_i, y_i, z_i):
            self.voxels[x_i, y_i, z_i] = value
        else:
            print("ERROR: Out of range (set_voxel_at_idx)")

    def get_voxel_at_idx(self, x_i, y_i, z_i):
        if self.idx_in_range(x_i, y_i, z_i):
            return self.voxels[x_i, y_i, z_i]
        else:
            print("ERROR: Out of range (get_voxel_at_idx)")
            return None

    def index_to_cartesian(self, xi, yi, zi):
        x = self.xmin + self.voxel_size * (xi + 0.5)
        y = self.ymin + self.voxel_size * (yi + 0.5)
        z = self.zmin + self.voxel_size * (zi + 0.5)
        return [x, y, z]

    def cartesian_to_idxs(self, x, y, z):
        x_idx = int((x - self.xmin) // self.voxel_size)
        y_idx = int((y - self.ymin) // self.voxel_size)
        z_idx = int((z - self.zmin) // self.voxel_size)
        return [x_idx, y_idx, z_idx]

    def cartesian_range_to_index_range(self, cartesian_range):
        '''
        format: [xmin, ymin, zmin, xmax, ymax, zmax]
        '''
        min_idx = self.cartesian_to_idxs(cartesian_range[0], cartesian_range[1], cartesian_range[2])
        max_idx = self.cartesian_to_idxs(cartesian_range[3], cartesian_range[4], cartesian_range[5])

        # clamp to be within grid
        for dim in range(len(self.voxels.shape)):
            if min_idx[dim] < 0:
                min_idx[dim] = 0
            if max_idx[dim] >= self.voxels.shape[dim]:
                max_idx[dim] = self.voxels.shape[dim] - 1
        return min_idx + max_idx

    def get_all_points_matching_value(self, value):
        '''
        Returns an Nx3 array where each row is an xyz coordinate matching value
        '''
        return self.get_all_points_in_range_matching_value(value, [self.xmin, self.xmax], [self.ymin, self.ymax], [self.zmin, self.zmax])

    def get_all_points_in_range_matching_value(self, value, x_range, y_range, z_range):
        '''
        Returns an Nx3 array where each row is an xyz coordinate matching value
        '''
        index_range = self.cartesian_range_to_index_range([x_range[0], y_range[0], z_range[0], x_range[1], y_range[1], z_range[1]])
        result = []
        for xi in range(index_range[0], index_range[3]+1):
            for yi in range(index_range[1], index_range[4]+1):
                for zi in range(index_range[2], index_range[5]+1):
                    if self.voxels[xi, yi, zi] == value:
                        result.append(self.index_to_cartesian(xi, yi, zi))
        return np.array(result)

    def cylinder_bounding_box(self, cylinder: Cylinder):
        '''
        Which cubic subset of the grid the cylinder is within
        '''
        cartesian_bounds = cylinder.get_bounding_box()
        return self.cartesian_range_to_index_range(cartesian_bounds)

    def update_points_within_cylinder(self, value, cylinder: Cylinder):
        bounds = self.cylinder_bounding_box(cylinder)
        for xi in range(bounds[0], bounds[3]+1):
            for yi in range(bounds[1], bounds[4]+1):
                for zi in range(bounds[2], bounds[5]+1):
                    point = self.index_to_cartesian(xi, yi, zi)
                    if cylinder.contains_point(point):
                        self.voxels[xi, yi, zi] = value

class BooleanGrid3D(Grid3D):
    def __init__(self, x_range, y_range, z_range, voxel_size):
        super().__init__(x_range, y_range, z_range, voxel_size, 1)

        # list of points whose value is 0
        self.changed_points = []
        self.num_free_start = 0

    @classmethod
    def init_from_file(cls, f):
        grid = None
        rangeline = f.readline().split(' ')
        resolutionline = f.readline().split(' ')
        shapeline = f.readline().split(' ')
        dataline = f.readline().split(' ')
        
        x_range = [float(rangeline[1]), float(rangeline[2])]
        y_range = [float(rangeline[3]), float(rangeline[4])]
        z_range = [float(rangeline[5]), float(rangeline[6])]
        voxel_size = float(resolutionline[1])
        shape = [int(shapeline[1]), int(shapeline[2]), int(shapeline[3])]

        grid = cls(x_range, y_range, z_range, voxel_size)
        data = np.array([int(datum) for datum in dataline[1:]])
        grid.voxels = data.reshape(shape)
        grid.changed_points = grid.get_all_points_matching_value(0)
        return grid

    def set_voxel_at_idx(self, x_i, y_i, z_i):
        if self.idx_in_range(x_i, y_i, z_i):
            if self.voxels[x_i, y_i, z_i]:
                self.changed_points.append(self.index_to_cartesian(x_i, y_i, z_i))
                self.voxels[x_i, y_i, z_i] = 0
        else:
            print("ERROR: Out of range (set_voxel_at_idx)")

    def mark_start_of_update(self):
        self.num_free_start = len(self.changed_points)

    def update_points_within_cylinder(self, cylinder: Cylinder):
        bounds = self.cylinder_bounding_box(cylinder)
        for xi in range(bounds[0], bounds[3]+1):
            for yi in range(bounds[1], bounds[4]+1):
                for zi in range(bounds[2], bounds[5]+1):
                    point = self.index_to_cartesian(xi, yi, zi)
                    if cylinder.contains_point(point):
                        self.set_voxel_at_idx(xi, yi, zi)

    def get_changed_points_since_update(self):
        return np.array(self.changed_points[self.num_free_start:])

    def get_num_free_cells(self):
        return len(self.changed_points)
