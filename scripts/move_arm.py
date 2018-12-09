#!/usr/bin/env python

import rospy
import actionlib
from arm_server.msg import SimpleTargetAction, SimpleTargetGoal

"""
this script simplify goals sending to arm_server_node
USAGE:
    import this script and use target_move(<x>, <y>, <z>, <frame>)
    option1:
        <frame>: the name of a link in robot description (IMPORTANT - starting with '/')
        <x>, <y>, <z>: coordinates relative to <frame> - for the arm to move to.
    option2:
        <frame>: a named target ('button','cobra_center','driving',ect.) for the arm to move to
            (IMPORTANT - not starting with '/')
        <x>, <y>, <z>: irrelevant
"""
# Called once when the goal completes
def target_done_callback(state, result):
    rospy.loginfo("[target_client]: finished in state [%d]", state)
    rospy.loginfo("[target_client]: answer - x: %f, y: %f, z: %f", result.x, result.y, result.z)


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
    goal.frame_id = frame_id

    # set target coordinates
    goal.x = x
    goal.y = y
    goal.z = z

    # send goal to action server
    target_client.send_goal(goal, target_done_callback, target_active_callback, target_feedback_callback)


if __name__ == '__main__':
    rospy.init_node('client_demo_node')
    rospy.loginfo("[client_demo]: started")

#    target_move(0, 0, 0, "cobra_center")
#    rospy.sleep(6)
    target_move(0, 0, 0, "driving")
