#!/usr/bin/env python

import rospy
import actionlib
from arm_server.msg import SimpleTargetAction, SimpleTargetGoal

# Called once when the goal completes
def target_done_callback(state, result):
    rospy.loginfo("[target_client]: finished in state [%d]", state)
    rospy.loginfo("[target_client]: answer - x: %f, y: %f, z: %f", result.x, result.y, result.z)
    # shutdown ros

# Called once when the goal becomes active
def target_active_callback():
    rospy.loginfo("[target_client]: goal just went active")


# Called every time feedback is received for the goal
def target_feedback_callback(feedback):
    rospy.loginfo("[target_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
                  feedback.x, feedback.y, feedback.z, feedback.distance)

def target_move(x, y, z, frame_id):
    target_client = actionlib.SimpleActionClient('simple_target', SimpleTargetAction)

    rospy.loginfo("[target_client]: waiting for target_server...")
    target_client.wait_for_server()
    rospy.loginfo("[target_client]: ready")

    # build goal
    goal = SimpleTargetGoal()
    goal.frame_id = frame_id # "/base_footprint" "/head_tilt_link" "/wrist_link"

    # set target coordinates
    goal.x = x  # 0.5 => 0.555
    goal.y = y  # 0.271 => 0.271
    goal.z = z  # 0.253 => -0.8535

    # send goal to action server
    target_client.send_goal(goal, target_done_callback, target_active_callback, target_feedback_callback)

if __name__ == '__main__':
    rospy.init_node('client_demo_node')
    rospy.loginfo("[client_demo]: started")

    target_move(0, 0, 0, "driving")
