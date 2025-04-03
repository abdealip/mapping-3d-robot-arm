import numpy as np

from camera import Camera
from joint_state_capture import JointTracker

class CameraJointTracker:
    def __init__(self, stationary_velocity_max_degrees_per_second=1) -> None:
        self.camera = Camera()
        self.joint_tracker = JointTracker()

        self.stationary_velocity_threshold = stationary_velocity_max_degrees_per_second * np.pi/180

    def snapshot_with_color(self):
        depth_image, color_image = self.camera.get_depth_and_rgb_image()

        joint_positions, joint_velocity = self.joint_tracker.get_joint_state()
        moving = sum(np.abs(joint_velocity)) > self.stationary_velocity_threshold

        if moving:
            print("Robot is moving too fast, not taking a snapshot")
            return (None, None, None)
        else:
            return (color_image, depth_image, joint_positions)

    def snapshot(self):
        # Convert images to numpy arrays
        depth_image = self.camera.get_depth_image()

        joint_positions, joint_velocity = self.joint_tracker.get_joint_state()
        moving = sum(np.abs(joint_velocity)) > self.stationary_velocity_threshold

        if moving:
            print("Robot is moving too fast, not taking a snapshot")
            return (None, None)
        else:
            return (depth_image, joint_positions)

    def cleanup(self):
        self.camera.cleanup()
        self.joint_tracker.cleanup()
