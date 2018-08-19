#!/usr/bin/env python

import rospy
import actionlib
from arm_server.msg import SimplePickAction, SimplePlaceAction, SimpleTargetAction, SimplePickGoal, SimplePlaceGoal, \
    SimpleTargetGoal

""" ################################################ PICK CALLBACKS ################################################"""


# Called once when the goal completes
def pick_done_callback(state, result):
    rospy.loginfo("[pick_client]: finished in state [%d]", state)
    rospy.loginfo("[pick_client]: answer - x: %f, y: %f, z: %f", result.x, result.y, result.z)
    # shutdown ros


# Called once when the goal becomes active
def pick_active_callback():
    rospy.loginfo("[pick_client]: goal just went active")


# Called every time feedback is received for the goal
def pick_feedback_callback(feedback):
    rospy.loginfo("[pick_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
                  feedback.x, feedback.y, feedback.z, feedback.distance)


""" ################################################ PLACE CALLBACKS ################################################"""


# Called once when the goal completes
def place_done_callback(state, result):
    rospy.loginfo("[place_client]: finished in state [%d]", state)
    rospy.loginfo("[place_client]: answer - x: %f, y: %f, z: %f", result.x, result.y, result.z)
    # shutdown ros


# Called once when the goal becomes active
def place_active_callback():
    rospy.loginfo("[place_client]: goal just went active")


# Called every time feedback is received for the goal
def place_feedback_callback(feedback):
    rospy.loginfo("[pick_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
                  feedback.x, feedback.y, feedback.z, feedback.distance)


""" ############################################### TARGET CALLBACKS ###############################################"""


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


""" ################################################################################################################"""


def pick_demo():
    pick_client = actionlib.SimpleActionClient('simple_pick', SimplePickAction)

    rospy.loginfo("[pick_client]: waiting for pick_server...")
    pick_client.wait_for_server()
    rospy.loginfo("[pick_client]: ready")

    # build goal
    goal = SimplePickGoal()
    goal.frame_id = "/base_footprint"
    goal.obj_name = "target"

    # set target coordinates
    goal.x = 0.7
    goal.y = 0.0
    goal.z = 0.6
    # set target cylinder primitives
    goal.h = 0.145
    goal.w = 0.03

    # send goal to action server
    pick_client.send_goal(goal, pick_done_callback, pick_active_callback, pick_feedback_callback)


def place_demo():
    place_client = actionlib.SimpleActionClient('simple_place', SimplePlaceAction)

    rospy.loginfo("[place_client]: waiting for pick_server...")
    place_client.wait_for_server()
    rospy.loginfo("[place_client]: ready")

    # build goal
    goal = SimplePlaceGoal()
    goal.frame_id = "/base_footprint"
    goal.obj_name = "target"

    # set target coordinates
    goal.x = 0.7
    goal.y = 0.0
    goal.z = 0.6

    # send goal to action server
    place_client.send_goal(goal, place_done_callback, place_active_callback, place_feedback_callback)


def target_demo(x, y, z, frame_id):
    target_client = actionlib.SimpleActionClient('simple_target', SimpleTargetAction)

    rospy.loginfo("[target_client]: waiting for pick_server...")
    target_client.wait_for_server()
    rospy.loginfo("[target_client]: ready")

    # build goal
    goal = SimpleTargetGoal()
    goal.frame_id = frame_id # "/base_footprint" "/head_pan_link"

    # set target coordinates
    goal.x = x  # 0.5
    goal.y = y  # 0.271
    goal.z = z  # 0.253

    # send goal to action server
    target_client.send_goal(goal, target_done_callback, target_active_callback, target_feedback_callback)

if __name__ == '__main__':
    rospy.init_node('client_demo_node')

    chosen = 0

    while (not rospy.is_shutdown()) and chosen != 9:
        print("Please choose one of the following actions:")
        print("1 - pick demo")
        print("2 - place demo")
        print("3 - target demo")
        print("9 - quit")

        chosen = input()
        if chosen == 1:
            rospy.loginfo("[client_demo]: executing pick demo")
            pick_demo()
        elif chosen == 2:
            rospy.loginfo("[client_demo]: executing place demo")
            place_demo()
        elif chosen == 3:
            rospy.loginfo("[client_demo]: executing target demo")
            x = input("x = ")
            y = input("y = ")
            z = input("z = ")
            frame_id = raw_input("frame_id = ")
            target_demo(x, y, z, frame_id)
        elif chosen == 9:
            rospy.loginfo("[client_demo]: exiting...")
            break
        else:
            rospy.logwarn("[client_demo]: Wrong input. Please choose valid option from menu")
