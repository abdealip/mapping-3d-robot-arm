import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
pipeline.start()

frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()

depth_data = depth.as_frame().get_data()
np_image = np.asanyarray(depth_data)

print(np_image)
