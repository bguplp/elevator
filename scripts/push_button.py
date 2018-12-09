#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan
from control_msgs.msg import JointControllerState
import move_arm

RAYS_NUM = 689
PRESSING_DISTANCE = 0.445
INTEL_BUTTON_LOC = 555


class push_button:
    """
    This class contains the functions and status,
    used by elevator_node to achieve it's goals.
    built for the dispatch mechanism
    """

    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.torso_pub = rospy.Publisher("/torso_effort_controller/command", Float64, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/URF/front", Range, self.urf_callback)
        rospy.Subscriber("/torso_effort_controller/state", JointControllerState, self.torso_callback)

        self.range = 2
        self.mid_scan = 2
        self.torso_height = 0
        self.status = 0
        self.counter_align = 0

    def urf_callback(self, data):
        self.range = "%.4f" % data.range

    def scan_callback(self, data):
        self.mid_scan = data.ranges[int(RAYS_NUM / 2)]

    def torso_callback(self, data):
        self.torso_height = "%.4f" % data.process_value

    def move_align(self, x, w):
        X = w / 2 - x

        if X > 30 or X < -30:
            self.counter_align = 0
            twist = Twist()
            if X < 0:
                twist.angular.z = -0.06
            else:
                twist.angular.z = 0.06

            self.vel_pub.publish(twist)

        else:
            self.counter_align += 1
            print("aligned {}%".format(self.counter_align * 10))
            if self.counter_align >= 10:
                self.status += 1
                self.counter_align = 0
                print("aligned!")
                rospy.sleep(2)

    def move_range(self):
        print("range = {}".format(self.range))
        if float(self.range) > PRESSING_DISTANCE:
            twist = Twist()
            twist.linear.x = 0.05 * float(self.range)
            self.vel_pub.publish(twist)
        else:
            print("in range!")
            self.status += 1

    def move_back(self, dist):
        print("range = {}".format(self.range))
        if float(self.range) < dist:
            twist = Twist()
            twist.linear.x = - 0.1 * (dist + 0.1 - float(self.range))
            self.vel_pub.publish(twist)
        else:
            self.status += 1
            print("moved back!")

    def move_torso(self, torso_h):
        self.torso_pub.publish(torso_h)

        while float(self.torso_height) < torso_h - 0.01 or float(self.torso_height) > torso_h + 0.055:
            print("waiting for torso")
            rospy.sleep(2)
        self.status += 1
        print("done moving torso")

    def push(self, x, pixel_size):
        dist_y = (INTEL_BUTTON_LOC - x) * pixel_size
        print("\033[1;32;40mpushing button. distance_y = {}\033[0m".format(dist_y))
        if dist_y > 0.04:
            dist_y = 0.04
        elif dist_y < -0.04:
            dist_y = -0.04

        dist_x = float(self.mid_scan) - 0.35
        print("pushing button. distance_x = {}".format(dist_x))
        if dist_x > 0.06:
            dist_x = 0.06

        move_arm.target_move(dist_x, dist_y, 0, "/wrist_link")
        rospy.sleep(3)
        move_arm.target_move(0, 0, 0, "button")
        rospy.sleep(3)
        self.status += 1
