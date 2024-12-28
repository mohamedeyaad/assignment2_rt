#!/usr/bin/env python3

import rospy
from assignment2_rt.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

# Global variable to store the last target
last_target = None

def handle_last_target_request(req):
    """Handle the service request and return the last target coordinates."""
    #global last_target
    if last_target:
        return LastTargetResponse(last_target.x, last_target.y)
    else:
        rospy.logwarn("No target set yet!")
        return LastTargetResponse(0.0, 0.0)  # Default response if no target is set

def target_callback(msg):
    """Callback function to update the last target from the subscribed topic."""
    global last_target
    last_target = msg
    rospy.loginfo(f"Last target updated: ({last_target.x}, {last_target.y})")

def main():
    """Initialize the service node."""
    global last_target

    rospy.init_node('service_node',anonymous=True)

    # Subscribe to the topic where the Action Client Node publishes the target
    rospy.Subscriber('/last_target', Point, target_callback)

    # Advertise the service
    rospy.Service('get_last_target', LastTarget, handle_last_target_request)
    rospy.loginfo("Service node ready to handle requests.")

    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
