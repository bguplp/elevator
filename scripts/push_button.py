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
        self.to_range = 1
        self.torso_height = 0
        self.status = 0

    def urf_callback(self, data):
        self.range = "%.4f" % data.range

    def torso_callback(self, data):
        self.torso_height = "%.4f" % data.process_value

    def move_align(self, x, w):
        X = w/2 - x
        print("X = {}".format(X))

        if X > 5 or X < -5:
            twist = Twist()
            if X < 0:
                twist.angular.z = -0.06
            else:
                twist.angular.z = 0.06

            self.vel_pub.publish(twist)

        else:
            self.status += 1
            print("aligned!")
            rospy.sleep(3)

    def move_range(self):
        if float(self.range) > self.to_range:
            print("range = {}".format(self.range))
            twist = Twist()
            twist.linear.x = 0.05 * float(self.range)
            self.vel_pub.publish(twist)
        else:
            print("range = {}".format(self.range))
            if self.to_range <= 0.65:
                print("in range!")
                self.status += 1
            else:
                self.to_range = 0.65
                self.status = 0

    def move_torso(self, torso_h):
        self.torso_pub.publish(torso_h)

        # real robot takes a lot of time
        while float(self.torso_height) < torso_h-0.01 or float(self.torso_height) > torso_h+0.05:
            print("waiting for torso")
            rospy.sleep(2)
        self.status += 1
        print("done moving torso")

    def push(self):
        # get closer than camera can see
        if float(self.range) > 0.47:
            print("final range = {}".format(self.range))
            twist = Twist()
            twist.linear.x = 0.05 * float(self.range)
            self.vel_pub.publish(twist)
            return

        print("pushing button. distance = {}".format(float(self.range)-0.32))
        move_arm.target_move(0.06, 0, 0, "/wrist_link")
        rospy.sleep(3)
        move_arm.target_move(-0.06, 0, 0, "/wrist_link")
        # move_arm.target_move(float(self.range)-0.32, 0, 0, "/wrist_link") # TODO change back
        # rospy.sleep(1)
        # move_arm.target_move(-float(self.range)+0.32, 0, 0, "/wrist_link")
        self.status += 1
