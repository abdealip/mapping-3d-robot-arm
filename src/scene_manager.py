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
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('scene_manager', anonymous=True)
        
        # Create planning scene interface
        self.scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)  # Wait for connection

        # Register services
        self.srv_clear = rospy.Service('scene_manager/clear_scene', Empty, self.handle_clear_scene)
        self.srv_table = rospy.Service('scene_manager/add_obstacle', Empty, self.handle_add_table)
        self.srv_wall = rospy.Service('scene_manager/add_wall', Empty, self.handle_add_wall)
        self.srv_complex = rospy.Service('scene_manager/add_complex', Empty, self.handle_add_complex)
        self.srv_mesh = rospy.Service('scene_manager/add_mesh', Empty, self.handle_add_mesh)
        self.srv_load_yaml = rospy.Service('scene_manager/load_yaml', SetBool, self.handle_load_yaml)
    
        rospy.loginfo("Scene manager services are now available.")
    
        
    def clear_scene(self):
        """Remove all objects from the scene"""
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)
        rospy.loginfo("Cleared all objects from scene")
        
    def add_box(self, name, position, size, orientation=None, frame_id="base_link"):
        """Add a box to the planning scene"""
        pose = self._create_pose(position, orientation)
        self.scene.add_box(name, pose, size=size)
        rospy.loginfo(f"Added box '{name}' at position {position}")
        
    def add_cylinder(self, name, position, height, radius, orientation=None, frame_id="base_link"):
        """Add a cylinder to the planning scene"""
        pose = self._create_pose(position, orientation)
        self.scene.add_cylinder(name, pose, height=height, radius=radius)
        rospy.loginfo(f"Added cylinder '{name}' at position {position}")
        
    def add_sphere(self, name, position, radius, frame_id="base_link"):
        """Add a sphere to the planning scene"""
        pose = self._create_pose(position)
        self.scene.add_sphere(name, pose, radius=radius)
        rospy.loginfo(f"Added sphere '{name}' at position {position}")
        
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
        
    def add_table(self):
        """Add a table in front of the robot"""
        self.add_box("table", [0.6, 0.0, -0.05], [0.8, 1.2, 0.05])
        
    def add_wall(self):
        """Add a wall to the side of the robot workspace"""
        self.add_box("wall", [0.5, 0.5, 0.5], [0.05, 1.0, 1.0])
        
    def add_complex_scene(self):
        """Add a more complex scene with multiple obstacles"""
        self.clear_scene()
        self.add_table()
        self.add_cylinder("cylinder1", [0.5, 0.3, 0.3], height=0.4, radius=0.05)
        self.add_sphere("sphere1", [0.7, -0.2, 0.3], radius=0.07)
        
    def load_mesh_scene(self):
        """Load a scene with mesh objects"""
        self.clear_scene()
        mesh_path = "package://khi_rs007l_moveit_config/meshes/obstacles/cnc.stl"
        self.add_mesh("mesh_obstacle", [0.6, 0.0, 0.3], mesh_path)
        
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
    
    def load_scene_from_yaml(self, yaml_file):
      """Load a scene from a YAML file"""
      import yaml
  
      try:
          with open(yaml_file, 'r') as file:
              scene_data = yaml.safe_load(file)
              
          if 'scene_objects' not in scene_data:
              rospy.logerr(f"Invalid scene file format: {yaml_file}")
              return False
              
          self.clear_scene()
          
          for obj in scene_data['scene_objects']:
              name = obj.get('name', 'unnamed')
              obj_type = obj.get('type', '')
              frame_id = obj.get('frame_id', 'base_link')
              
              # Get pose
              pose_data = obj.get('pose', {})
              position = pose_data.get('position')
              orientation = pose_data.get('orientation')
              
              if obj_type == 'box':
                  dimensions = obj.get('dimensions', [0.1, 0.1, 0.1])
                  self.add_box(name, position, dimensions, orientation, frame_id)
                  
              elif obj_type == 'cylinder':
                  height = obj.get('height', 0.1)
                  radius = obj.get('radius', 0.1)
                  self.add_cylinder(name, position, height, radius, orientation, frame_id)
                  
              elif obj_type == 'sphere':
                  radius = obj.get('radius', 0.1)
                  self.add_sphere(name, position, radius, frame_id)
                  
              elif obj_type == 'mesh':
                  mesh_file = obj.get('mesh_file')
                  scale = obj.get('scale')
                  self.add_mesh(name, position, mesh_file, orientation, scale, frame_id)
                  
              else:
                  rospy.logwarn(f"Unknown object type: {obj_type} for object {name}")
                  
          rospy.loginfo(f"Loaded scene from {yaml_file}")
          return True
        
      except Exception as e:
        rospy.logerr(f"Error loading scene file {yaml_file}: {str(e)}")
        return False

    # Service handlers
    def handle_clear_scene(self, req):
        self.clear_scene()
        return EmptyResponse()
        
    def handle_add_table(self, req):
        self.add_table()
        return EmptyResponse()
        
    def handle_add_wall(self, req):
        self.add_wall()
        return EmptyResponse()
        
    def handle_add_complex(self, req):
        self.add_complex_scene()
        return EmptyResponse()
        
    def handle_add_mesh(self, req):
        self.load_mesh_scene()
        return EmptyResponse()

    def handle_load_yaml(self, req):
      """Service handler to load a scene from YAML file"""
      yaml_path = "/home/rob-barton-group/catkin_ws/src/khi_robot/khi_rs007l_moveit_config/config/scenes/lab.yaml"
      success = self.load_scene_from_yaml(yaml_path)
      return SetBoolResponse(success=success, message="Scene loaded successfully" if success else "Failed to load scene")
      



def main():
    argv = rospy.myargv(argv=sys.argv)
    
    parser = ArgumentParser()
    # Keep your existing arguments
    parser.add_argument('--clear', action='store_true', help='Clear the scene')
    parser.add_argument('--table', action='store_true', help='Add a table')
    parser.add_argument('--wall', action='store_true', help='Add a wall')
    parser.add_argument('--complex', action='store_true', help='Add a complex scene')
    parser.add_argument('--mesh', action='store_true', help='Add a mesh')
    parser.add_argument('--lab_scene', action='store_true', help='Load lab scene')
    args = parser.parse_args(argv[1:])
    
    scene_manager = SceneManager()
    
    if args.clear:
        scene_manager.clear_scene()
        return
    
    if args.table:
        scene_manager.add_table()
        
    if args.wall:
        scene_manager.add_wall()
        
    if args.complex:
        scene_manager.add_complex_scene()
        
    if args.mesh:
        scene_manager.load_mesh_scene()

    if args.lab_scene:
        yaml_path = "/home/rob-barton-group/catkin_ws/src/khi_robot/khi_rs007l_moveit_config/config/scenes/lab.yaml"
        scene_manager.load_scene_from_yaml(yaml_file=yaml_path)
        
    # If no arguments provided, show help
    if not (args.clear or args.table or args.wall or args.complex or args.mesh):
        parser.print_help()
    
    # Keep the node alive
    rospy.loginfo("Scene manager is running. Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass