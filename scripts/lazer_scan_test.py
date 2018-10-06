#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Range

urf_range = 0


def urf_callback(data):
    global urf_range
    urf_range = data.range
    # print("data.range = {}".format(data.range))


def scan_callback(data):
    range_sum = 0
    min = data.range_min
    max = data.range_max
    max_p = min
    min_p = max
    left = -1
    right = -1

    for i in range(0, 720):
        if min < data.ranges[i] < max:
            range_sum += data.ranges[i]

            if right == -1:
                right = data.ranges[i]

            if data.ranges[i] > max_p:
                max_p = data.ranges[i]
            if data.ranges[i] < min_p:
                min_p = data.ranges[i]

    for i in range(719, -1, -1):
        if min < data.ranges[i] < max:
            left = data.ranges[i]
            break

    print("-=(MIN + MAX) / 2=-")
    print("min_p = {0}, max_p = {1}".format(min_p, max_p))
    print("mid = {}".format((min_p + max_p) / 2))
    print("----------------------------------------------")
    print("-=LEFT-MOST and RIGHT-MOST=-")
    print("left = {0}, right = {1}".format(left, right))
    print("mid = {}".format((left + right) / 2))
    print("----------------------------------------------")
    print("-=URF RANGE=-")
    print("urf_range = {}".format(urf_range))
    print("----------------------------------------------")
    print("-=SUM / NUM_OF_RAYS=-")
    print("range_sum/720 = {}".format(range_sum / 720))
    print("----------------------------------------------")
    print("-=MIDDLE RAY=-")
    print("data.ranges[360] = {}".format(data.ranges[360]))
    print("//////////////////////////////////////////////")


rospy.init_node('laser_listener', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scan_callback)
rospy.Subscriber("/urf/front", Range, urf_callback)
rospy.spin()
