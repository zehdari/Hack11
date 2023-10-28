import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import nav_msgs.msg
from std_msgs.msg import Empty, String
import geometry_msgs.msg
from cv_bridge import CvBridge
import threading
import time

class TelloController():
    def __init__(self, node):
        super().__init__()

        self.node = node

        self.title("DJI Tello Controller")

        # DJ SPIN THAT
        self.ros_spin_thread = threading.Thread(target=rclpy.spin, args=[self.node])
        self.ros_spin_thread.start()

        self.bridge = CvBridge()

        # Is moving flag
        self.is_moving = False

        # ROS2 publishers
        self.takeoff_pub = self.node.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.node.create_publisher(Empty, '/land', 1)
        self.emergency_pub = self.node.create_publisher(Empty, '/emergency', 1)
        self.control_pub = self.node.create_publisher(geometry_msgs.msg.Twist, '/control', 1)
        self.prec_move_pub = self.node.create_publisher(geometry_msgs.msg.Pose, '/prec_control', 1)
        self.node.create_subscription(sensor_msgs.msg.BatteryState, "/battery", self.update_battery_status, 10)
        self.flip_pub = self.node.create_publisher(String, '/flip', 1)

        # ROS2 subscribers
        self.odom_sub = self.node.create_publisher(nav_msgs.msg.Odometry, 'odom', 1)

        self.takeoff()
        self.move_to_pose(1, 0, 0)
        self.land()

    def takeoff(self):
        msg = Empty()
        self.takeoff_pub.publish(msg)

    def land(self):
        msg = Empty()
        self.land_pub.publish(msg)

    def emergency(self):
        msg = Empty()
        self.emergency_pub.publish(msg)

    def move_forward(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.y = 100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def move_backward(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.y = -100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def move_left(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def move_right(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def rotate_left(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = -100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def rotate_right(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 100.0
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)
    
    def move_up(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.z = 100.0  
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def move_down(self, button=False):
        twist = geometry_msgs.msg.Twist()
        twist.linear.z = -100.0 
        self.control_pub.publish(twist)
        if button:
            self.schedule_stop(1.0)

    def stop_movement(self, key=None):
        twist = geometry_msgs.msg.Twist()
        
        if key == 'w' or key == 's':
            twist.linear.y = 0.0
            print("y = 0")
        elif key == 'a' or key == 'd':
            twist.linear.x = 0.0
            print("x = 0")
        elif key == 'r' or key == 'f':
            twist.linear.z = 0.0
            print("z = 0")
        elif key == 'q' or key == 'e':
            twist.angular.z = 0.0
            print("ang = 0")
        else:
            print("all = 0")
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        self.control_pub.publish(twist)

    def move_to_pose(self, x, y, z, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)

        self.prec_move_pub.publish(pose)

    

def main(args=None):
    rclpy.init(args=args)

    node = Node('tello_gui_controller')

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()