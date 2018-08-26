#! /usr/bin/env python

import rospy
import actionlib
import math
from moveit_commander.move_group import MoveGroupCommander
import elevator.msg
from geometry_msgs.msg import PoseStamped
from tf.listener import TransformListener


class arm_server_node(object):
    # create messages that are used to publish feedback/result
    _feedback = elevator.msg.SimpleTargetFeedback()
    _result = elevator.msg.SimpleTargetResult()

    def __init__(self, name):
        # wait for moveit
        while not "/move_group/result" in dict(rospy.get_published_topics()).keys():
            rospy.sleep(2)

        self.group = MoveGroupCommander("arm")
        self.group.set_start_state_to_current_state()
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_pose_reference_frame("/base_footprint")
        self.group.set_max_velocity_scaling_factor(1)
        self.group.set_num_planning_attempts(50)
        self.group.set_planning_time(5)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.02)

        self.tf_listener = TransformListener()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, elevator.msg.SimpleTargetAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):

        eef_pose = self.group.get_current_pose("gripper_link")

        self._feedback.x = math.fabs(goal.x - eef_pose.pose.position.x)
        self._feedback.y = math.fabs(goal.y - eef_pose.pose.position.y)
        self._feedback.z = math.fabs(goal.z - eef_pose.pose.position.z)
        self._feedback.distance = math.sqrt(
            math.pow(self._feedback.x, 2) + math.pow(self._feedback.y, 2) + math.pow(self._feedback.z, 2))
        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # start executing the action
        if goal.frame_id[0] != '/':
            rospy.loginfo('[%s]: Executing, moving arm to "%s" pose' % (self._action_name, goal.frame_id))
            self.group.set_named_target(goal.frame_id)
            named_plan = self.group.plan()
            self.group.execute(named_plan)

        else:
            rospy.loginfo('[%s]: Executing, moving gripper from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)' % (
                self._action_name, eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z, goal.x,
                goal.y, goal.z))

            origin_goal = PoseStamped()
            origin_goal.header.frame_id = "/gripper_link"
            origin_goal.pose.position.x = goal.x #+ eef_pose.pose.position.x
            origin_goal.pose.position.y = goal.y #+ eef_pose.pose.position.y
            origin_goal.pose.position.z = goal.z #+ eef_pose.pose.position.z
            # origin_goal.pose.orientation = eef_pose.pose.orientation
            transformed_goal = self.tf_listener.transformPose("/base_footprint", origin_goal)
            transformed_goal.pose.orientation.x = 0
            transformed_goal.pose.orientation.y = 0
            transformed_goal.pose.orientation.z = 0
            transformed_goal.pose.orientation.w = 0
            self.group.set_pose_target(transformed_goal)
            # print("transformed_goal: {}".format(transformed_goal))
            plan = self.group.plan()
            self.group.execute(plan)


if __name__ == '__main__':
    rospy.init_node('arm_server_node')
    server = arm_server_node('simple_target')
    rospy.spin()
