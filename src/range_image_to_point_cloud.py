#!/usr/bin/python3

from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
from freespace_plotter import *

class RangeImagePointCloud:
    def __init__(self, fov=(87, 58), range=[17, 5000], downsample=10):
        self.fov = fov
        self.range = range
        self.downsample = downsample

    def point_cloud_from_range_image(self, depth_image):

        horizontal_degrees_per_pixel = self.fov[0]/depth_image.shape[1]
        vertical_degrees_per_pixel = self.fov[1]/depth_image.shape[0]
        points = []

        for row_i, row in enumerate(depth_image):
            if row_i % self.downsample != 0:
                continue
            for col_i, depth_value in enumerate(row):
                if col_i % self.downsample != 0:
                    continue
                if depth_value < self.range[0] or depth_value > self.range[1]:
                    continue
                horizontal_angle = (self.fov[0]/2 - col_i * horizontal_degrees_per_pixel) * np.pi/180
                vertical_angle = (self.fov[1]/2 - row_i * vertical_degrees_per_pixel) * np.pi/180

                # print(row_i, col_i, depth_value)
                # print(horizontal_angle)
                # print(vertical_angle)

                # xz_plane_distance = depth_value #* np.cos(vertical_angle)
                # z = xz_plane_distance # * np.cos(horizontal_angle)
                # x = -xz_plane_distance * np.sin(horizontal_angle)
                # y = -depth_value * np.tan(vertical_angle)
                z = depth_value
                y = z*np.tan(vertical_angle)
                x = z*np.tan(horizontal_angle)
                points.append([-z/1000, -x/1000, y/1000])
        points = np.array(points)

        return points

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Input Depth Image")

    options = parser.parse_args()

    depth_image = np.loadtxt(options.input)

    ripc = RangeImagePointCloud()
    points = ripc.point_cloud_from_range_image(depth_image)

    plotter = FreespacePlotter(ViewEnum.ISO, "Test", [-5, 5], [-5, 5], [-5, 5], interactive=False)

    plotter.add_points(points, "Depth Points")
    plotter.update()

    plt.savefig("test.png")
