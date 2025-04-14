#!/usr/bin/env python

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
import tf
import sys
import os
from argparse import ArgumentParser

from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse

class SceneManager:
    def __init__(self, mesh_path):
        # Initialize ROS node
        rospy.init_node('scene_manager', anonymous=True)
        
        # Create planning scene interface
        self.scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)  # Wait for connection

        # Register services
        self.srv_mesh = rospy.Service('scene_manager/add_mesh', Empty, self.handle_add_mesh)

        rospy.loginfo("Scene manager services are now available.")

        self.mesh_path = mesh_path
        
    def clear_scene(self):
        """Remove all objects from the scene"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        rospy.loginfo("Cleared all objects from scene")
        
    # def add_mesh(self, name, position, filename, orientation=None, scale=(1,1,1), frame_id="base_link"):
    #     """Add a mesh to the planning scene from an STL/DAE file"""
    #     pose = self._create_pose(position, orientation)
        
    #     # Handle both absolute paths and package:// paths
    #     if not filename.startswith("package://"):
    #         # Check if file exists
    #         if not os.path.exists(filename):
    #             rospy.logerr(f"Mesh file not found: {filename}")
    #             return False
        
    #     self.scene.add_mesh(name, pose, filename, scale)
    #     rospy.loginfo(f"Added mesh '{name}' from {filename}")

    def add_mesh(self, name, position, filename, orientation=None, scale=(1,1,1), frame_id="base_link"):
      """Add a mesh to the planning scene from an STL/DAE file"""
      pose = self._create_pose(position, orientation)
      
      # Handle package:// paths
      if filename.startswith("package://"):
          import rospkg
          rp = rospkg.RosPack()
          package_name = filename.split("package://")[1].split("/")[0]
          try:
              package_path = rp.get_path(package_name)
              filename = filename.replace(f"package://{package_name}", package_path)
              rospy.loginfo(f"Resolved mesh path: {filename}")
          except rospkg.ResourceNotFound:
              rospy.logerr(f"Package '{package_name}' not found")
              return False
      
      # Verify file exists
      if not os.path.exists(filename):
          rospy.logerr(f"Mesh file not found: {filename}")
          return False
      
      # Try to add the mesh with better error reporting
      try:
          self.scene.add_mesh(name, pose, filename, scale)
          rospy.loginfo(f"Added mesh '{name}' from {filename}")
          return True
      except Exception as e:
          rospy.logerr(f"Failed to add mesh '{name}': {str(e)}")
          return False

    def load_mesh_scene(self):
        self.clear_scene()
        self.add_mesh("environment", [0.6, 0.0, 0.3], self.mesh_path)

    def _create_pose(self, position, orientation=None):
        """Helper method to create a PoseStamped from position and orientation
        
        Parameters:
            position: [x, y, z] position coordinates
            orientation: Either [x, y, z, w] quaternion or [roll, pitch, yaw] Euler angles in radians
        """
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        
        # Handle orientation if provided
        if orientation is not None:
            # Check if we have Euler angles (3 values) or quaternion (4 values)
            if len(orientation) == 3:
                # Convert Euler angles (RPY) to quaternion
                quaternion = tf.transformations.quaternion_from_euler(
                    orientation[0], orientation[1], orientation[2])
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
            elif len(orientation) == 4:
                # Use quaternion directly
                pose.pose.orientation.x = orientation[0]
                pose.pose.orientation.y = orientation[1]
                pose.pose.orientation.z = orientation[2]
                pose.pose.orientation.w = orientation[3]
            else:
                rospy.logwarn("Invalid orientation format: must be [roll, pitch, yaw] or [x, y, z, w]")
                pose.pose.orientation.w = 1.0  # Default orientation
        else:
            pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
            
        return pose

    def handle_add_mesh(self, req):
        self.load_mesh_scene()
        return EmptyResponse()

def main():
    argv = rospy.myargv(argv=sys.argv)
    
    parser = ArgumentParser()
    parser.add_argument('--mesh', required=True, help='STL file representing lab environment')
    args = parser.parse_args(argv[1:])
    
    scene_manager = SceneManager(args.mesh)
            
    scene_manager.load_mesh_scene()
    
    # Keep the node alive
    rospy.loginfo("Scene manager is running. Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass