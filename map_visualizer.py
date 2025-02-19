from grid3D import Grid3D
import matplotlib.pyplot as plt

grid = Grid3D.init_from_file("test.map")
occupied_points = grid.get_all_points_matching_value(1)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(occupied_points[:, 0], occupied_points[:, 1], occupied_points[:, 2])

plt.show()
