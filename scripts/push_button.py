#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from control_msgs.msg import JointControllerState
import move_arm


class push_button:

    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.torso_pub = rospy.Publisher("/torso_effort_controller/command", Float64, queue_size=10)

        rospy.Subscriber("/urf/front", Range, self.urf_callback)
        rospy.Subscriber("/torso_effort_controller/state", JointControllerState, self.torso_callback)

        self.range = 2
        self.torso_height = 0
        self.status = 0
        # self.align_again = 1
        self.counter_align = 0

    def urf_callback(self, data):
        self.range = "%.4f" % data.range

    def torso_callback(self, data):
        self.torso_height = "%.4f" % data.process_value

    def move_align(self, x, w):
        X = w / 2 - x - 30
        if X > 15 or X < -15:
            self.counter_align = 0
            twist = Twist()
            if X < 0:
                twist.angular.z = -0.06
            else:
                twist.angular.z = 0.06

            self.vel_pub.publish(twist)

        else:
            self.counter_align += 1
            print("checking alignment")
            if self.counter_align >= 10:
                self.status += 1
                self.counter_align = 0
                print("aligned!")

    def move_range(self):
        print("range = {}".format(self.range))
        if float(self.range) > 0.45:
            twist = Twist()
            twist.linear.x = 0.05 * float(self.range)
            self.vel_pub.publish(twist)
        else:
            print("in range!")
            self.status += 1

    def move_torso(self, torso_h):
        self.torso_pub.publish(torso_h)

        # real robot takes a lot of time
        while float(self.torso_height) < torso_h - 0.01 or float(self.torso_height) > torso_h + 0.05:
            print("waiting for torso")
            rospy.sleep(2)
        self.status += 1
        print("done moving torso")

    def push(self, x, w):
        X = w / 2 - x

        # get closer than camera can see
        # if self.align_again:
        #     self.status = 0
        #     self.align_again = 0
        #     return
        dist = float(self.range) - 0.32
        print("pushing button. distance = {}".format(dist))
        if dist > 0.06:
            dist = 0.06

        move_arm.target_move(dist, 0, 0, "/wrist_link")
        rospy.sleep(3)
        move_arm.target_move(-dist, 0, 0, "/wrist_link")
        rospy.sleep(3)
        self.status += 1
