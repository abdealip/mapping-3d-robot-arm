import matplotlib.cm
import matplotlib.figure
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

class Camera:
    def __init__(self, start_pipeline=True) -> None:
        viridis_cmap = plt.cm.viridis
        colors = viridis_cmap(np.linspace(0, 1, viridis_cmap.N))
        colors[0] = [0, 0, 0, 1]
        self.colormap = ListedColormap(colors)

        if start_pipeline:
            self.start_capture()
        else:
            self.pipeline = None

    def start_capture(self):
        self.pipeline = rs.pipeline()
        self.pipeline.start()

    def get_depth_image(self):
        if self.pipeline == None:
            self.start_capture()
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        return np.asanyarray(depth_frame.get_data())
    
    def get_rgb_image(self):
        if self.pipeline == None:
            self.start_capture()
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def get_depth_and_rgb_image(self):
        if self.pipeline == None:
            self.start_capture()
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        return (np.asanyarray(depth_frame.get_data()), np.asanyarray(color_frame.get_data()))

    def plot_depth_image(self, depth_image, fig=None):
        fig: matplotlib.figure.Figure
        if fig == None:
            fig = plt.gcf()
        ax = fig.gca()
        ax.pcolormesh(depth_image, cmap=self.colormap, shading='auto')
        ax.invert_yaxis()
        ax.set_xlabel('Column index')
        ax.set_ylabel('Row index')
        ax.set_title("Depth Image")
        fig.colorbar(matplotlib.cm.ScalarMappable(matplotlib.colors.Normalize(vmin=np.min(depth_image.reshape(-1)), vmax = np.max(depth_image.reshape(-1))), cmap=self.colormap), label='Value')

    def plot_color_image(self, color_image, fig=None):
        fig: matplotlib.figure.Figure
        if fig == None:
            fig = plt.gcf()
        ax = fig.gca()
        ax.imshow(color_image)
        ax.set_title("RGB Image")

    def save_depth_image(self, depth_image, out_file):
        np.savetxt(depth_image, out_file)

    def save_rgb_image(self, color_image, out_file):
        image = Image.fromarray(color_image)
        image.save(out_file)

    def cleanup(self):
        self.pipeline.stop()

if __name__ == "__main__":
    camera = Camera()
    depth_image = camera.get_depth_image()
    color_image = camera.get_rgb_image()

    plt.figure(constrained_layout=True)
    plt.subplot(2, 1, 1)
    camera.plot_depth_image(depth_image)

    plt.subplot(2, 1, 2)
    camera.plot_color_image(color_image)

    plt.show()
