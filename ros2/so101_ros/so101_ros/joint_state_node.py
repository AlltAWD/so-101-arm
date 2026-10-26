import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        timer_period = 0.05 # seconds. Here we want 20Hz so time interval should be 0.05s
        self.timer = self.create_timer(timer_period, self.timer_callback)

        

        

    def timer_callback(self):
        joints = self.arm.read_joints_rad()
        if None in joints.values():
            return          # a servo didn't answer; skip this cycle
        self.get_logger().debug('joints: "%s"' % joints)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # nested field: msg.header.stamp
        msg.name = list(joints.keys())     # list[str]
        msg.position = list(joints.values())  # list[float], radians, same order as name
        self.publisher_.publish(msg)

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