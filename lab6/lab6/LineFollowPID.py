#!/usr/bin/env python3

import sys
import cv2
import numpy
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, '/camera1/image_raw', self.process_data, 1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        # timer_period = 0.2
        # self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.get_logger().info("Node Started!")

    def send_cmd_vel(self):
        kp = 0.025
        ki = 0.0
        kd = 0.0

        self.integral += self.error
        derivative = self.error - self.prev_error
        self.prev_error = self.error
        control_signal = kp * self.error + ki * self.integral + kd * derivative
        self.velocity.linear.x = 0.12
        self.velocity.angular.z = control_signal
        self.publisher.publish(self.velocity)

    def process_data(self, data):
        self.get_logger().info("Image Received!")
        frame = self.bridge.imgmsg_to_cv2(data)
        light_line = numpy.array([0, 0, 0])
        dark_line = numpy.array([260, 100, 100])
        mask = cv2.inRange(frame, light_line, dark_line)
        r1 = 140
        c1 = 0
        img = mask[r1:r1+300, c1:c1+512]
        cnts, _ = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) == 0:
            pass
        else:
            max_contour = max(cnts, key=cv2.contourArea)
            rect=cv2.minAreaRect(max_contour)
            self.error = 90-rect[2]
            if abs(self.error) > 25:
                self.error = 25 if self.error > 0 else -25
            print(self.error)
            if self.error==0:
                return
            self.send_cmd_vel()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
