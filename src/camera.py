import pyrealsense2 as rs
import numpy as np
from PIL import Image

pipeline = rs.pipeline()
pipeline.start()
frames = pipeline.wait_for_frames()
pipeline.stop()

depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Convert images to numpy arrays
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

np.savetxt("frame1_depth.txt", depth_image)

# Create a PIL Image object
image = Image.fromarray(color_image)

# Save the image
image.save("frame1_color.png")
