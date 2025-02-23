import os
from grid3D import Grid3D
import matplotlib.pyplot as plt

base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
config_file = os.path.join(base_dir, "config", "map_config.json")
map_file = os.path.join(base_dir, "data", "test.map")

grid = Grid3D.init_from_file(map_file)
occupied_points = grid.get_all_points_matching_value(1)
not_occupied_points = grid.get_all_points_matching_value(0)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

legend = []
# ax.scatter(occupied_points[:, 0], occupied_points[:, 1], occupied_points[:, 2])
# legend.append("Occupied")

ax.scatter(not_occupied_points[:, 0], not_occupied_points[:, 1], not_occupied_points[:, 2])
legend.append("Free")

plt.legend(legend)
plt.title("Free Space")

ax.axes.set_xlim3d(-0.5, 1.0)
ax.axes.set_ylim3d(-0.5, 1.0)
ax.axes.set_zlim3d(-0.5, 1.0)
ax.axes.set_box_aspect((1, 1, 1))

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

ax.view_init(elev=30, azim=45, roll=0)

plt.show()
