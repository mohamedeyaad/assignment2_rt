#!/usr/bin/env python3

"""
Service Node for the assignment2_rt package.

This node provides a ROS service (`get_last_target`) to retrieve the last target coordinates
set by the action client. It subscribes to the `/last_target` topic to update the target
coordinates and responds to service requests with the most recent target.
"""

import rospy
from assignment2_rt.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

# Global variable to store the last target
last_target = None
"""geometry_msgs.msg.Point: Stores the last target coordinates received from the `/last_target` topic."""

def handle_last_target_request(req):
    """
    Handle the service request and return the last target coordinates.

    :param req: The service request (not used in this implementation).
    :type req: assignment2_rt.srv.LastTargetRequest
    :return: The last target coordinates (x, y).
    :rtype: assignment2_rt.srv.LastTargetResponse
    """
    if last_target:
        return LastTargetResponse(last_target.x, last_target.y)
    else:
        rospy.logwarn("No target set yet!")
        return LastTargetResponse(0.0, 0.0)  # Default response if no target is set

def target_callback(msg):
    """
    Callback function to update the last target from the subscribed topic.

    :param msg: The Point message containing the target coordinates.
    :type msg: geometry_msgs.msg.Point
    """
    global last_target
    last_target = msg
    rospy.loginfo(f"Last target updated: ({last_target.x}, {last_target.y})")

def main():
    """
    Initialize the service node.

    This function sets up the ROS node, subscribes to the `/last_target` topic,
    and advertises the `get_last_target` service. It keeps the node running
    using `rospy.spin()`.
    """
    global last_target

    rospy.init_node('service_node', anonymous=True)

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
