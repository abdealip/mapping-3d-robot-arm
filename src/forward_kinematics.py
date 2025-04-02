from typing import List
from enum import IntEnum
import numpy as np

from cylinder import Cylinder

def joint_angles_close(position1, position2, threshold):
    if position1 == None or position2 == None:
        return False
    
    if (len(position1) != len(position2)):
        return False
    
    for i in range(len(position1)):
        if abs(position1[i] - position2[i]) > threshold:
            return False
    
    return True

class TwistType(IntEnum):
    REVOLUTE = 0,
    PRISMATIC = 1,

class Twist:
    def __init__(self, twist):
        '''
        twist format: [v1, v2, v3, w1, w2, w3]
        '''
        v = twist[:3]
        w = twist[3:]
        self.type = None
        if np.linalg.norm(w) > 0:
            w = w / np.linalg.norm(w)
            self.type = TwistType.REVOLUTE
        elif np.linalg.norm(v) > 0:
            v = v / np.linalg.norm(v)
            self.type = TwistType.PRISMATIC
        else:
            print("ERROR: zero joint twist")
            return None

        self.v = v
        self.w = w
        self.w_hat = np.array([[    0, -w[2], w[1] ],
                               [ w[2],     0, -w[0]],
                               [-w[1],  w[0],     0]])
        self.w_hat_sq = self.w_hat @ self.w_hat

    def matrix_exponential(self, theta):
        if self.type == TwistType.PRISMATIC:
            ret = np.identity(4)
            ret[:3, 3] = self.v * theta
            return ret
        elif self.type == TwistType.REVOLUTE:
            so3_transform = np.identity(3) + self.w_hat * np.sin(theta) + self.w_hat_sq * (1 - np.cos(theta))
            translational_part = (np.identity(3) - so3_transform) @ np.cross(self.w, self.v) + self.w * np.dot(self.w, self.v) * theta
            ret = np.identity(4)
            ret[:3, :3] = so3_transform
            ret[:3, 3] = translational_part
            return ret
        else:
            return None

class ForwardKinematics:
    def __init__(self, twists: List[Twist], reference_config: np.ndarray):
        self.twists = twists
        self.reference_config = reference_config

    def body_to_base_tf(self, joint_angles):
        if len(joint_angles) < len(self.twists):
            print("ERROR: not enough joint angles")
            return
        body_to_base_tf = np.identity(4)
        for i in range(len(self.twists)):
            body_to_base_tf = body_to_base_tf @ self.twists[i].matrix_exponential(joint_angles[i])
        body_to_base_tf = body_to_base_tf @ self.reference_config
        return body_to_base_tf

class TransformableCylinder(Cylinder):
    def __init__(self, radius, length, body_frame_direction, reference_config, twists: List[Twist]):
        '''
        radius: cylinder radius
        length: cylinder length
        body_frame_direction: cylinder axis in body frame
        reference_config: SE(3) element (4x4 array) describing pose of cylinder in base frame with 0 joint angles
        twists: array of twists to apply to cylinder
        '''
        super().__init__(radius, length)
        self.body_frame_direction = np.append(np.array(body_frame_direction) / np.linalg.norm(body_frame_direction), 0)
        self.reference_config = np.array(reference_config)
        self.fk = ForwardKinematics(twists, reference_config)
        self.transform(np.zeros(len(twists)))

    def transform(self, joint_angles):
        body_to_base_tf = self.fk.body_to_base_tf(joint_angles)
        base_point = body_to_base_tf @ np.array([0, 0, 0, 1])
        direction = body_to_base_tf @ self.body_frame_direction
        self.set_base_point(base_point[:3])
        self.set_direction(direction[:3])
