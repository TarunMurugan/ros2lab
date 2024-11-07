#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ZigZag(Node):
    def __init__(self):
        super().__init__('ZigZag')
        self.get_logger().info("To stop, press ctrl + c")
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.zigzag_state = 0
    
    def timer_callback(self):
        msg = Twist()
        if self.zigzag_state == 0:
            msg.linear.x = 0.1
            msg.angular.z = 0.5
        else:
            msg.linear.x = 0.1
            msg.angular.z = -0.5
        self.publisher_.publish(msg)
        self.zigzag_state = 1 - self.zigzag_state

def main(args=None):
    rclpy.init(args=args)
    node = ZigZag()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
