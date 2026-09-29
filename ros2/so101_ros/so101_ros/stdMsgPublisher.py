import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class stdMsgPublihser(Node):

    def __init__(self):
        super().__init__('std_msg_publisher') # Just like Java's OOP, this super calls the `Node` class's constructor and named the node 'std_msg_publihser' 
        self.publisher_ = self.create_publisher(String, 'topic', 10) # 'create_publisher' declares the node publishes String messages over a topic 'topic'
        timer_period = 1 # seconds. Here we want 1Hz so time interval should be 1s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # A counter used in callback

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    std_msg_publisher = stdMsgPublihser()

    rclpy.spin(std_msg_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    std_msg_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

