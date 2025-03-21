#!/usr/bin/env python3

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from geometry_msgs.msg import Twist
from assignment2_rt.msg import RobotState
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import messagebox

# Global variables
robot_velocity = (0.0, 0.0)
"""tuple: The current velocity of the robot (linear, angular)."""

robot_position = (0.0, 0.0)
"""tuple: The current position of the robot (x, y)."""

pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
"""rospy.Publisher: Publisher for the robot's state."""

def odom_callback(msg):
    """
    Callback to update robot position and velocity.

    This function is triggered whenever a new Odometry message is received.
    It updates the global variables `robot_position` and `robot_velocity`
    and publishes the robot's state.

    :param msg: The Odometry message containing the robot's position and velocity.
    :type msg: nav_msgs.msg.Odometry
    """
    global robot_position, robot_velocity, pub
    robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    robot_velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
    robot_data = RobotState()
    robot_data.x, robot_data.y = robot_position
    robot_data.vel_x, robot_data.vel_z = robot_velocity
    pub.publish(robot_data)
    
def send_goal(client, x, y):
    """
    Send a goal to the action server.

    This function creates a `PlanningGoal` message with the specified
    target position and sends it to the action server.

    :param client: The action client used to send the goal.
    :type client: actionlib.SimpleActionClient
    :param x: The x-coordinate of the goal position.
    :type x: float
    :param y: The y-coordinate of the goal position.
    :type y: float
    """
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal, feedback_cb=feedback_callback)

def feedback_callback(feedback):
    """Handle feedback from the action server."""
    rospy.loginfo(f"Feedback: {feedback}")

def set_goal():
    """Set goal from tkinter input."""
    try:
        x = float(goal_x_entry.get())
        y = float(goal_y_entry.get())
        send_goal(client, x, y)
        new_target = Point()
        new_target.x = x
        new_target.y = y
        target_pub.publish(new_target)
        result_label.config(text=f"Goal set to ({x}, {y})", fg="green")
    except ValueError:
        result_label.config(text="Invalid input. Please enter valid numbers.", fg="red")

def cancel_goal():
    """Cancel the current goal."""
    client.cancel_goal()
    result_label.config(text="Goal canceled.", fg="red")

def main():
    """Main function for the action client."""
    global client, target_pub, goal_x_entry, goal_y_entry, result_label
    rospy.init_node('action_client_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    target_pub = rospy.Publisher('/last_target', Point, queue_size=10)
    
    # Action client setup
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server!")
    
    # Tkinter setup
    root = tk.Tk()
    root.title("Robot Goal Control")
    root.geometry("500x280")

    input_frame = tk.Frame(root)
    input_frame.pack(pady=20)

    tk.Label(input_frame, text="Enter X Goal:", font=("Arial", 12)).grid(row=0, column=0, padx=10, pady=5)
    goal_x_entry = tk.Entry(input_frame, font=("Arial", 12))
    goal_x_entry.grid(row=0, column=1, padx=10, pady=5)

    tk.Label(input_frame, text="Enter Y Goal:", font=("Arial", 12)).grid(row=1, column=0, padx=10, pady=5)
    goal_y_entry = tk.Entry(input_frame, font=("Arial", 12))
    goal_y_entry.grid(row=1, column=1, padx=10, pady=5)

    tk.Button(root, text="Set Goal", font=("Arial", 14), bg="#4CAF50", fg="white", command=set_goal).pack(pady=10)
    tk.Button(root, text="Cancel Goal", font=("Arial", 14), bg="#FF5733", fg="white", command=cancel_goal).pack(pady=10)

    result_label = tk.Label(root, text="", font=("Arial", 12))
    result_label.pack(pady=10)

    root.mainloop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass