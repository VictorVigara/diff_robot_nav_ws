#!/usr/bin/env python

import rospy
import actionlib
import goal_action.msg 
from geometry_msgs.msg import Pose, Point, Quaternion

def navigate_client():
    client = actionlib.SimpleActionClient('navigate', goal_action.msg.goalAction)
    client.wait_for_server()

    goal = goal_action.msg.goalGoal()

    # Example waypoints
    waypoint1 = Pose(Point(1.0,2.5, 0.0), Quaternion(0.0, 0.0, 0.75, 0.66))
    waypoint2 = Pose(Point(-1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    goal.waypoints = [waypoint1, waypoint2]

    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f" Final status: {result.result}")

def feedback_callback(feedback):

    rospy.loginfo(f"Feedback path progress: {feedback.progress} %")
    rospy.loginfo(f"Feedback distance next waypoint: {feedback.distance_next_waypoint} m")


if __name__ == '__main__':
    try:
        rospy.init_node('navigate_client')
        result = navigate_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
