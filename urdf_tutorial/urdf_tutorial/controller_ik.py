#!/usr/bin/env python3
#colcon build --packages-select urdf_tutorial 
#ros2 run urdf_tutorial controller --ros-args -p end_location:=[3.5,1.5,-1.2]
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
class TrajectoryPublisher(Node):
 
    def __init__(self):
        super().__init__('trajectory_node')
        topic_ = "/joint_trajectory_controller/joint_trajectory"
        self.joints = ['base_arm1_joint', 'arm1_arm2_joint', 'arm2_arm3_joint']
        self.declare_parameter("end_params", [0.5, 0.5, 1.2])
        self.goal_=self.ik_solver(self.get_parameter("end_params").value)
        self.publisher_ = self.create_publisher(JointTrajectory, topic_, 10)
        self.timer_ = self.create_timer(1,self.timer_callback)

    def ik_solver(self, end_location):
        # Lengths of the arm segments
        l1 = 0.5
        l2 = 0.5
        l3 = 0.3

        # Extract end location coordinates
        y,x, end_effector_angle = end_location

        # Calculate the distance from the base to the end effector
        d = math.sqrt(x**2 + y**2)

        # Check if the end location is reachable
        if d > l1 + l2 + l3:
            self.get_logger().warn("End location is out of reach!")
            return None
        y_rect = y - l3 * math.sin(end_effector_angle)
        x_rect = x - l3 * math.cos(end_effector_angle)

        # Calculate the angles for each joint
        
        theta2 = math.acos((x_rect**2 + y_rect**2 - d**2) / (2 * l1 * l2))
        theta1 = math.atan2(y_rect, x_rect)-math.atan2(l2*math.sin(theta2),l1+l2*math.cos(theta2))
        theta3 = end_effector_angle - theta1 - theta2
        
        return [theta1, theta2, theta3]
    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        self.publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
