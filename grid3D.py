import numpy as np

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

    def write_to_file(self, filename):
        with open(filename, "w") as outf:
            lines = []
            lines.append(f"range: {self.xmin} {self.xmax} {self.ymin} {self.ymax} {self.zmin} {self.zmax}\n")
            lines.append(f"resolution: {self.voxel_size}\n")
            lines.append(f"shape: {self.xdim} {self.ydim} {self.zdim}\n")
            data = self.voxels.flatten()
            data_str = " ".join([str(datum) for datum in data])
            lines.append(f"data: {data_str}\n")
            outf.writelines(lines)

    @classmethod
    def init_from_file(self, filename):
        grid = None
        with open(filename, "r") as f:
            lines = f.readlines()
            rangeline = lines[0].split(' ')
            resolutionline = lines[1].split(' ')
            shapeline = lines[2].split(' ')
            dataline = lines[3].split(' ')
            
            x_range = [float(rangeline[1]), float(rangeline[2])]
            y_range = [float(rangeline[3]), float(rangeline[4])]
            z_range = [float(rangeline[5]), float(rangeline[6])]
            voxel_size = float(resolutionline[1])
            shape = [int(shapeline[1]), int(shapeline[2]), int(shapeline[3])]

            grid = Grid3D(x_range, y_range, z_range, voxel_size, 0)
            data = np.array([int(datum) for datum in dataline[1:]])
            grid.voxels = data.reshape(shape)
        return grid

    def in_range(self, x, y, z) -> bool:
        return (x > self.xmin and x < self.xmax and y > self.ymin and y < self.ymax and z > self.zmin and z < self.zmax)

    def cartesian_to_idxs(self, x, y, z):
        x_idx = (x - self.xmin) // self.voxel_size
        y_idx = (y - self.ymin) // self.voxel_size
        z_idx = (z - self.zmin) // self.voxel_size
        return (x_idx, y_idx, z_idx)

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

    def index_to_cartesian()

    def get_all_points_matching_value(self, value):
        '''
        Returns an Nx3 array where each row is an xyz coordinate matching value
        '''
        result = []
        for xi in range(self.xdim):
            for yi in range(self.ydim):
                for zi in range(self.zdim):
                    if self.voxels[xi, yi, zi] == value:
                        x = self.xmin + self.voxel_size * (xi + 0.5)
                        y = self.ymin + self.voxel_size * (yi + 0.5)
                        z = self.zmin + self.voxel_size * (zi + 0.5)
                        result.append([x, y, z])
        return np.array(result)

    def set_all_points_within_cylinder_to_value(self, value, cylinder_base_point, axis, length, radius):
        '''
        value: integer value
        cylinder_base_point: 3D point in base frame
        axis: vector describing axis of cylinder in base frame
        length: length of cylinder
        radius: radius of cylinder
        '''

        pass
