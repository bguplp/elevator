#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import move_arm

class push_button:

    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.torso_pub = rospy.Publisher("/torso_effort_controller/command", Float64, queue_size=10)
        self.urf_sub = rospy.Subscriber("/URF/front", Range, self.urf_callback)
        self.range = 0
        self.status = 0

    def urf_callback(self, data):
        self.range = "%.4f" % data.range

    def move_align(self, x, w):
        X = w/2 - x
        if X > 15 or X < -15:
            twist = Twist()
            twist.angular.z = -0.06
            # if X < 30:
            #     twist.angular.z = X / w * 2
            # else:
            #     twist.angular.z = X / w / 2
            self.vel_pub.publish(twist)
            print("X = {0}, vel = {1}".format(X, twist.angular.z))
        else:
            self.status += 1
            print("aligned!")

    def move_range(self):
        if float(self.range) > 0.4:
            print("range = {}".format(self.range))
            twist = Twist()
            twist.linear.x = 0.05 * float(self.range)
            self.vel_pub.publish(twist)
        else:
            print("in range!")
            self.status += 1

    def move_torso(self, torso_h):
        rospy.sleep(1)
        self.torso_pub.publish(torso_h)
        # real robot takes a lot of time
        rospy.sleep(30)
        self.status += 1

    def push(self):
        print("pushing button. distance = {}".format(float(self.range)-0.3))
        move_arm.target_move(float(self.range)-0.32, 0, 0, "/wrist_link")
        rospy.sleep(1)
        move_arm.target_move(-float(self.range)+0.32, 0, 0, "/wrist_link")
        self.status += 1
