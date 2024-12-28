#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from geometry_msgs.msg import Twist
from assignment2_rt.msg import RobotState

robot_velocity = (0.0, 0.0)
robot_position = (0.0, 0.0)
pub = rospy.Publisher('/custom_msg', RobotState, queue_size=10)

def odom_callback(msg):
    """Callback to update robot position and velocity."""
    #rospy.loginfo(f"Odometry: {msg}")
    robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    robot_velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
    robot_data = RobotState()
    robot_data.x, robot_data.y = robot_position
    robot_data.vel_x, robot_data.vel_z = robot_velocity
    pub.publish(robot_data)
    
def send_goal(client, x, y):
    """Send a goal to the action server."""
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal, feedback_cb=feedback_callback)

def feedback_callback(feedback):
    """Handle feedback from the action server."""
    rospy.loginfo(f"Feedback: {feedback}")

def check_goal_status(client):
    """Check the status and result of the goal."""
    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")
        result = client.get_result()
        rospy.loginfo(f"Result: {result}")
    elif state == GoalStatus.ABORTED:
        rospy.loginfo("Goal aborted.")
    elif state == GoalStatus.REJECTED:
        rospy.loginfo("Goal rejected.")
    else:
        rospy.loginfo(f"Goal in state {state}")

def main():
    """Main function for the action client."""
    rospy.init_node('action_client_node',anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    #rate = rospy.Rate(10)
    
    # Action client setup
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server!")
    

    while not rospy.is_shutdown():
        # Publish robot state
        robot_state = RobotState()
        robot_state.x, robot_state.y = robot_position
        robot_state.vel_x, robot_state.vel_z = robot_velocity
        pub.publish(robot_state)

        # Simple user interface for setting/canceling goals
        command = input("Enter 'set x y' to set goal or 'cancel' to cancel: ").strip()
        if command.startswith("set"):
            _, x, y = command.split()
            send_goal(client, float(x), float(y))
            #client.wait_for_result()  # Wait for the result of the goal
            #check_goal_status(client)  # Check the goal status and result
        elif command == "cancel":
            client.cancel_goal()
            rospy.loginfo("Goal canceled.")
        else:
            rospy.logwarn("Unknown command.")
        #rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
