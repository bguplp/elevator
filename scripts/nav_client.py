#!/usr/bin/env python

import rospy
import math
import move_arm

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class move_base_client:

    def __init__(self, points, angles):

        rospy.init_node('move_base_client')

        if not points:
            points_seq = rospy.get_param('move_base_client/point_seq')
            yaw_seq = rospy.get_param('move_base_client/yaw_seq')
        else:
            points_seq = points
            yaw_seq = angles

        # move arm to 'driving' position
        move_arm.target_move(0, 0, 0, "driving")
        rospy.sleep(3)
        # List of goal quaternions:
        quat_seq = list()
        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        for yaw in yaw_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yaw * math.pi / 180, axes='sxyz'))))

        n = 3
        # Returns a list of lists [[point1], [point2],...,[pointn]]
        points = [points_seq[i:i + n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n - 3]))
            n += 1

        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose %d is now being processed by the Action Server..." % (self.goal_cnt + 1))

    def feedback_cb(self, feedback):
        rospy.loginfo("going to pose %d.\ncurrent location: (%.2f, %.2f)" % (self.goal_cnt + 1, feedback.base_position.pose.position.x, feedback.base_position.pose.position.y))

    def done_cb(self, status, result):
        self.goal_cnt += 1

        if status == 2:
            rospy.loginfo("Goal pose %d received a cancel request after it started executing, completed execution!" % self.goal_cnt)

        if status == 3:
            rospy.loginfo("Goal pose %d reached" % self.goal_cnt)
            self.next_goal()

        if status == 4:
            rospy.loginfo("Goal pose %d was aborted by the Action Server" % self.goal_cnt)
            self.next_goal()

        if status == 5:
            rospy.loginfo("Goal pose %d has been rejected by the Action Server" % self.goal_cnt)
            self.next_goal()

        if status == 8:
            rospy.loginfo("Goal pose %d received a cancel request before it started executing, successfully cancelled!" % self.goal_cnt)

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose %d to Action Server" % (self.goal_cnt + 1))
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

    def next_goal(self):
        if self.goal_cnt < len(self.pose_seq):
            next_goal = MoveBaseGoal()
            next_goal.target_pose.header.frame_id = "map"
            next_goal.target_pose.header.stamp = rospy.Time.now()
            next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose %d to Action Server" % (self.goal_cnt + 1))
            rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
            self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
        else:
            rospy.loginfo("Final goal pose reached!")
            rospy.signal_shutdown("Final goal pose reached!")
            return


if __name__ == '__main__':
    try:
        move_base_client([], [])
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
