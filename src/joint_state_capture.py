import rospy
from sensor_msgs.msg import JointState
import threading
import numpy as np
import time

def spin_thread():
    rospy.spin()

class JointTracker:
    def __init__(self) -> None:

        rospy.init_node("joint_listener", anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.joint_handler)

        self.spin_thread_obj = threading.Thread(target=spin_thread)
        self.spin_thread_obj.start()

        self.joint_lock = threading.Lock()

        self.last_joint_pose = []
        self.last_joint_velocity = []

    def joint_handler(self, data: JointState):
        self.joint_lock.acquire()
        self.last_joint_pose = data.position
        self.last_joint_velocity = data.velocity
        self.joint_lock.release()

    def get_joint_state(self):
        self.joint_lock.acquire()
        joint_positions = np.copy(self.last_joint_pose)
        joint_velocity = np.copy(self.last_joint_velocity)
        self.joint_lock.release()
        return (joint_positions, joint_velocity)

    def cleanup(self):
        self.spin_thread_obj.join()

if __name__ == "__main__":
    jt = JointTracker()

    while not rospy.is_shutdown():
        pos, vel = jt.get_joint_state()
        print("positions:", pos)
        print("velocities:", vel)
        time.sleep(0.1)
    
    jt.cleanup()
