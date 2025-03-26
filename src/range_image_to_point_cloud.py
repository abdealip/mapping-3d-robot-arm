#!/usr/bin/python3

from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
from freespace_plotter import *

parser = ArgumentParser()
parser.add_argument("-i", "--input", required=True, help="Input Depth Image")

options = parser.parse_args()

depth_image = np.loadtxt(options.input)

FOV = (87, 58)  # FOV in degrees (H, V)

horizontal_degrees_per_pixel = FOV[0]/depth_image.shape[1]
vertical_degrees_per_pixel = FOV[1]/depth_image.shape[0]

RANGE = [17, 5000]

points = []

DOWNSAMPLE = 10

for row_i, row in enumerate(depth_image):
    if row_i % DOWNSAMPLE != 0:
        continue
    for col_i, depth_value in enumerate(row):
        if col_i % DOWNSAMPLE != 0:
            continue
        if depth_value < RANGE[0] or depth_value > RANGE[1]:
            continue
        horizontal_angle = (FOV[0]/2 - col_i * horizontal_degrees_per_pixel) * np.pi/180
        vertical_angle = (FOV[1]/2 - row_i * vertical_degrees_per_pixel) * np.pi/180

        # print(row_i, col_i, depth_value)
        # print(horizontal_angle)
        # print(vertical_angle)

        xz_plane_distance = depth_value * np.cos(vertical_angle)
        z = xz_plane_distance * np.cos(horizontal_angle)
        x = -xz_plane_distance * np.sin(horizontal_angle)
        y = -depth_value * np.sin(vertical_angle)
        points.append([-z/1000, y/1000, x/1000])

    #     if len(points) > 1000:
    #         break
    
    # if len(points) > 1000:
    #     break

print(len(points))
points = np.array(points)

plotter = FreespacePlotter(ViewEnum.ISO, "Test", [-5, 0], [-2.5, 2.5], [-2.5, 2.5], interactive=False)

plotter.add_points(points, "Depth Points")
plotter.update()

plt.show()
