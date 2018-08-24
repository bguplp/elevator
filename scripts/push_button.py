#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import move_arm

class push_button:

    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.torso_pub = rospy.Publisher("/torso_position_controller/command", Float64, queue_size=10)
        self.urf_sub = rospy.Subscriber("/URF/front", Range, self.urf_callback)
        self.range = 0
        self.status = 0
        self.move = [self.move_align, self.move_range, self.move_torso, self.push]

    def urf_callback(self, data):
        self.range = "%.4f" % data.range

    def move_align(self, x, w, torso_h):
        X = x - w/2
        if self.status == 0 and X > 5 or X < -5:
            twist = Twist()
            twist.angular.z = -0.003*X
            self.vel_pub.publish(twist)
        else:
            self.status = 1
            print("aligned!")

    def move_range(self, x, w, torso_h):
        if float(self.range) > 0.4:
            print("range = {}".format(self.range))
            twist = Twist()
            twist.linear.x = 0.1 * float(self.range)
            self.vel_pub.publish(twist)
        else:
            print("in range!")
            self.status = 2

    def move_torso(self, x, w, torso_h):
        rospy.sleep(2)
        print("torso")
        self.torso_pub.publish(torso_h)
        rospy.sleep(2)
        self.status = 3

    def push(self, x, w, torso_h):
        print("init push!")
        move_arm.target_move(0.06, 0, 0, "/wrist_link")
        rospy.sleep(3)
        move_arm.target_move(-0.06, 0, 0, "/wrist_link")
        rospy.sleep(3)
        self.status = 4
