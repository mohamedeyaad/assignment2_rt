#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from geometry_msgs.msg import Twist

def send_goal(client, x, y):
    """Send a goal to the action server."""
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal, feedback_cb=feedback_callback)

def feedback_callback(feedback):
    """Handle feedback from the action server."""
    rospy.loginfo(f"Feedback: {feedback}")

def main():
    """Main function for the action client."""
    rospy.init_node('action_client_node',anonymous=True)
    #rate = rospy.Rate(1)
    
    # Action client setup
    client = actionlib.SimpleActionClient('reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server!")
   
    while not rospy.is_shutdown():
         # Simple user interface for setting/canceling goals
        command = input("Enter 'set x y' to set goal or 'cancel' to cancel: ").strip()
        if command.startswith("set"):
            _, x, y = command.split()
            send_goal(client, float(x), float(y))
        elif command == "cancel":
            client.cancel_goal()
            rospy.loginfo("Goal canceled.")
        else:
            rospy.logwarn("Unknown command.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
