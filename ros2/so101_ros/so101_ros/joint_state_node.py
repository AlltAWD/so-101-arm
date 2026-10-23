import sys
from pathlib import Path
import rclpy
from rclpy.node import Node

# colcon copies this file into ~/ros2_ws/install/, so a __file__-relative path
# would point there. The repo has a fixed home, so use it directly.
sys.path.insert(0, str(Path.home() / "so-101-arm"))

from servo.arm import Arm, FOLLOWER

class JointStateNode(Node):

    def __init__(self):
        super().__init__('joint_state_node') # Just like Java's OOP, this super calls the `Node` class's constructor and named the node 'joint_state_node'
        self.arm = Arm(FOLLOWER) 
        self.arm.open()
        self.arm.load_calibration()
        self.arm.torque(False)
        
        timer_period = 0.05 # seconds. Here we want 20Hz so time interval should be 0.05s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        
        joints = self.arm.read_joints_rad()
        
        self.get_logger().info('joints: "%s"' % joints)
        

def main(args=None):
    rclpy.init(args=args)

    joint_state_node = JointStateNode()
    try:
        rclpy.spin(joint_state_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_node.arm.close()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        joint_state_node.destroy_node()
        rclpy.try_shutdown()
        
    


if __name__ == '__main__':
    main()