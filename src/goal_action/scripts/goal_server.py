#!/usr/bin/env python

import rospy
import actionlib
import goal_action.msg
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class NavigationServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('navigate', goal_action.msg.goalAction, self.execute, False)
        self.server.start()
        self.move_base_client =  actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.feedback = goal_action.msg.goalFeedback()
        self.progress = 0.0

    def execute(self, goal):
        
        result = goal_action.msg.goalResult()

        self.waypoints = goal.waypoints
        total_waypoints = len(self.waypoints)
        success = True
        for i, waypoint in enumerate(self.waypoints):

            self.curr_wayp = i
            prog_perc = ((i+1)/len(self.waypoints))*100
            rospy.loginfo(f"Moving to waypoint {i+1}/{total_waypoints}")
                

            # Create move base goal
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.header.stamp = rospy.Time.now()
            move_base_goal.target_pose.pose = waypoint


            # Send goal to move base server
            self.move_base_client.send_goal(move_base_goal, feedback_cb=self.feedback_cb)

            # Wait for the server to finish the action 
            wait = self.move_base_client.wait_for_result()

            if not wait:
                rospy.logerr("Move base action server not available!")
                success = False
                break
            else:
                move_base_result = self.move_base_client.get_result()
                if not move_base_result:
                    rospy.logerr("Move base failed to reach the goal")
                    success = False
                    break

            # Update and publish feedback
            self.progress = prog_perc
            self.feedback.progress = self.progress
            self.server.publish_feedback(self.feedback)


            # Check for preempt (cancellation)
            if self.server.is_preempt_requested():
                rospy.loginfo('Navigation preempted')
                self.server.set_preempted()
                success = False
                break

        if success:
            result.result = String(data="Navigation completed")
            rospy.loginfo("Navigation completed successfully.")
            self.server.set_succeeded(result)
        else:
            result.result = String(data="ERROR")
            rospy.loginfo("Navigation was preempted or failed.")
            self.server.set_aborted(result)

    def feedback_cb(self, feedback_move_base):

        self.x_pos = feedback_move_base.base_position.pose.position.x
        self.y_pos = feedback_move_base.base_position.pose.position.y
        
        next_waypoint = self.waypoints[self.curr_wayp]
        next_wayp_x = next_waypoint.position.x
        next_wayp_y = next_waypoint.position.y

        distance_next_waypoint = math.sqrt((self.x_pos-next_wayp_x)**2+(self.y_pos-next_wayp_y)**2)

        feedback_navigation = goal_action.msg.goalFeedback()
        feedback_navigation.distance_next_waypoint = distance_next_waypoint
        feedback_navigation.progress = self.progress
        self.server.publish_feedback(feedback_navigation)

        
if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
