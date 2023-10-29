import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
from std_msgs.msg import Empty, String
import nav_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge
import threading
import time

class TelloController(tk.Tk):
    def __init__(self, node):
        super().__init__()

        self.node = node

        self.title("DJI Tello Controller")

        # DJ SPIN THAT
        self.ros_spin_thread = threading.Thread(target=rclpy.spin, args=[self.node])
        self.ros_spin_thread.start()

        self.bridge = CvBridge()

        # ROS2 publishers
        self.takeoff_pub = self.node.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.node.create_publisher(Empty, '/land', 1)
        self.emergency_pub = self.node.create_publisher(Empty, '/emergency', 1)
        self.control_pub = self.node.create_publisher(geometry_msgs.msg.Twist, '/control', 1)
        self.prec_move_pub = self.node.create_publisher(geometry_msgs.msg.Pose, '/prec_move', 1)
        self.flip_pub = self.node.create_publisher(String, '/flip', 1)

        # ROS2 subscribers
        self.node.create_subscription(sensor_msgs.msg.BatteryState, "/battery", self.update_battery_status, 10)
        self.node.create_subscription(nav_msgs.msg.Odometry, "/run_slam/camera_pose", self.cb_odom, 1)

        # Keypress
        self.bind("<Key>", self.key_press)
        self.bind("<KeyRelease>", self.key_release)

        self.pressed_keys = set()
        self.last_press_time = {}
        self.last_spam_time = {}
        
        # BUTTONS :O
        self.takeoff_button = ttk.Button(self, text="Takeoff (T)", command=self.takeoff)
        self.takeoff_button.grid(row=2, column=0, padx=10, pady=10)

        self.land_button = ttk.Button(self, text="Land (L)", command=self.land)
        self.land_button.grid(row=2, column=1, padx=10, pady=10)

        self.emergency_button = ttk.Button(self, text="Emergency (H)", command=self.emergency)
        self.emergency_button.grid(row=2, column=2, padx=10, pady=10)

        self.stop_button = ttk.Button(self, text="Stop", command=self.stop_movement)
        self.stop_button.grid(row=2, column=3, padx=10, pady=10) 

        self.move_up_button = ttk.Button(self, text="Up (R)", command=self.move_up(button=True))
        self.move_up_button.grid(row=0, column=3, padx=10, pady=10)

        self.move_down_button = ttk.Button(self, text="Down (F)", command=self.move_down(button=True))
        self.move_down_button.grid(row=1, column=3, padx=10, pady=10)

        self.rotate_left_button = ttk.Button(self, text="Rotate Left (Q)", command=self.rotate_left(button=True))
        self.rotate_left_button.grid(row=0, column=0, padx=10, pady=10)

        self.forward_button = ttk.Button(self, text="Forward (W)", command=self.move_forward(button=True))
        self.forward_button.grid(row=0, column=1, padx=10, pady=10)

        self.rotate_right_button = ttk.Button(self, text="Rotate Right (E)", command=self.rotate_right(button=True))
        self.rotate_right_button.grid(row=0, column=2, padx=10, pady=10)

        self.left_button = ttk.Button(self, text="Left (A)", command=self.move_left(button=True))
        self.left_button.grid(row=1, column=0, padx=10, pady=10)

        self.backward_button = ttk.Button(self, text="Backward (S)", command=self.move_backward(button=True))
        self.backward_button.grid(row=1, column=1, padx=10, pady=10)

        self.right_button = ttk.Button(self, text="Right (D)", command=self.move_right(button=True))
        self.right_button.grid(row=1, column=2, padx=10, pady=10)

        self.battery_label = ttk.Label(self, text="Battery: --%")
        self.battery_label.grid(row=3, column=0, columnspan=3, padx=10, pady=10)

        self.test_move = ttk.Button(self, text="GOOOO", command=self.test_move)
        self.test_move.grid(row=4, column=0, padx=10, pady=10)

        self.pose = [0.0, 0.0, 0.0]

    def test_move(self):
        self.takeoff()
        time.sleep(5)
        while(self.is_moving):
            time.sleep(0.5)
        print("moving")
        self.move_to_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068)
        time.sleep(5)
        self.land()

    def schedule_stop(self, secs):
        threading.Timer(secs, self.stop_movement).start()

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
    

    def key_press(self, event):
        key = event.keysym.lower()
        now = time.time()
        if key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self.last_press_time[key] = now
            print("Press:", key)

        if key == 'w':
            self.move_forward()
        elif key == 's':
            self.move_backward()
        elif key == 'a':
            self.move_left()
        elif key == 'd':
            self.move_right()
        elif key == 'q':
            self.rotate_left()
        elif key == 'e':
            self.rotate_right()
        elif key == 'r':
            self.move_up()
        elif key == 'f':
            self.move_down()
        elif key == 't':
            self.takeoff()
        elif key == 'l':
            self.land()
        elif key == 'h':
            self.emergency()

    def key_release(self, event, recursion=False):
        key = event.keysym.lower()
        now = time.time()
        if key in self.pressed_keys:
            if now - self.last_press_time[key] > 0.075:
                self.pressed_keys.remove(key)
                print("Release1:", key)
            else:
                if not recursion:
                    self.last_press_time[key] = now
                    self.last_spam_time[key] = now
                    self.after(125, self.key_release, event, True)
                return
        else:
            return
        self.stop_movement(key)


    def update_battery_status(self, battery_data):
        battery_percentage = int(battery_data.percentage)
        self.battery_label.config(text=f"Battery: {battery_percentage}%")

    def move_to_pose(self, x, y, z, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.prec_move_pub.publish(pose)

    def cb_odom(self, msg: nav_msgs.msg.Odometry):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = msg.pose.pose.position.z

def main(args=None):
    rclpy.init(args=args)

    node = Node('tello_gui_controller')
    app = TelloController(node)
    app.mainloop()

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()