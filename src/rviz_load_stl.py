#!/usr/bin/python3

import os
import rospy
from argparse import ArgumentParser
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface

def create_pose(position):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = rospy.Time.now()
    
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.w = 1.0
        
    return pose

class SceneManager:
    def __init__(self):
        rospy.init_node('scene_manager', anonymous=True)
        self.scene = PlanningSceneInterface(synchronous=True)
        
    def clear_scene(self):
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)

    def add_mesh(self, name, filename):
        pose = create_pose([0, 0, 0])
        self.scene.add_mesh(name, pose, filename)

if __name__ == '__main__':    
    parser = ArgumentParser()
    parser.add_argument('-m', '--mesh', required=True, help='STL file representing lab environment')
    args = parser.parse_args()
    
    scene_manager = SceneManager()

    if not os.path.exists(args.mesh):
        rospy.logerr(f"Mesh file {args.mesh} not found, clearing scene")
        scene_manager.clear_scene()
    else:
        scene_manager.add_mesh("environment", args.mesh)
