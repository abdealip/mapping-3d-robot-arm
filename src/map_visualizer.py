import os
from grid3D import Grid3D
import matplotlib.pyplot as plt

base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
config_file = os.path.join(base_dir, "config", "map_config.json")
map_file = os.path.join(base_dir, "data", "test.map")

grid = Grid3D.init_from_file(map_file)
# occupied_points = grid.get_all_points_matching_value(1)

not_occupied_points = grid.get_all_points_matching_value(0)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# ax.scatter(occupied_points[:, 0], occupied_points[:, 1], occupied_points[:, 2])
ax.scatter(not_occupied_points[:, 0], not_occupied_points[:, 1], not_occupied_points[:, 2])

ax.axes.set_xlim3d(-0.5, 2.5)
ax.axes.set_ylim3d(-0.5, 2.5)
ax.axes.set_zlim3d(-0.5, 2.5)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

plt.show()
