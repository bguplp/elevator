#!/usr/bin/env python

import roslib
import sys
import os
import math
import rospy
import cv2
from sensor_msgs.msg import Image
from control_msgs.msg import GripperCommandActionGoal
from cv_bridge import CvBridge, CvBridgeError
from button_finder import Button_finder
from push_button import push_button
import move_arm


class armadillo_elevator_node:

    def __init__(self):
        self.button_img = rospy.get_param('~button_img')
        self.pressed_button_img = rospy.get_param('~pressed_button_img')
        self.press_offset_x = float(rospy.get_param('~press_offset_x'))
        self.press_offset_y = float(rospy.get_param('~press_offset_y'))

        self.image_pub = rospy.Publisher("image_buttons", Image, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper_controller/gripper_cmd/goal", GripperCommandActionGoal,
                                           queue_size=10)

        self.pb = push_button()
        self.push_ready = 0
        self.counter = 0

        self.move_control = [self.move_control0, self.move_control1, self.move_control2,
                             self.move_control3, self.move_control4, self.move_control5, self.move_control6,
                             self.move_control7, self.move_control8, self.move_control9, self.move_control10]
        self.status_str = ["searching button", "searching button", "align robot to button",
                           "move towards button", "move towards button", "adjust height to button",
                           "pushing button", "lower robot", "check if pressed", "check if pressed",
                           "going into elevator"]
                           
        # move arm to 'button' position
        move_arm.target_move(0, 0, 0, "button")
        rospy.sleep(5)

        # close gripper
        gc = GripperCommandActionGoal()
        gc.goal.command.max_effort = 0.1
        self.gripper_pub.publish(gc)
        rospy.sleep(2)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/softkinetic/rgb/image_raw", Image, self.camera_callback)

        self.scale = -1
        self.detected = 0
        self.threshold = -1
        self.button_location = (-1, -1)
        self.button_height = -1
        self.button_width = -1

    def camera_callback(self, data):
        if self.pb.status == 11:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img_height, img_width = cv_image.shape[:2]

        print("STATUS: {}".format(self.status_str[self.pb.status]))
        self.move_control[self.pb.status](cv_image, img_width)

        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.detected:
            cv2.putText(cv_image, 'button detected', (img_width - 300, 25), font, 0.8, (0, 153, 0), 2)
            cv2.putText(cv_image, 'certainty: {}%'.format(self.threshold * 100), (img_width - 300, 50), font, 0.8,
                        (0, 153, 0), 2)
            if self.pb.status != 3:
                cv2.rectangle(cv_image, self.button_location, (
                    self.button_location[0] + self.button_width, self.button_location[1] + self.button_height),
                              (255, 255, 0), 2)

        else:
            cv2.putText(cv_image, 'button not detected', (img_width - 300, 25), font, 0.8, (0, 0, 255), 2)

        cv2.imshow("button detector", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def update_match(self, cv_image, scale_min, scale_max, temp_img, threshold):
        bf = Button_finder(cv_image)
        self.scale, \
        (self.detected, \
         self.threshold, \
         self.button_location, \
         self.button_height, \
         self.button_width) = bf.find_match_multi_size(scale_min, scale_max, temp_img, threshold)

    def move_control0(self, cv_image, img_width):
        self.update_match(cv_image, 0.2, 1.8, self.button_img, 0.7)
        self.pb.status = 1

    def move_control1(self, cv_image, img_width):
        if self.detected:
            self.pb.status = 2
        else:
            self.counter += 1
            self.pb.move_align(-10, img_width)
            if self.counter >= 30:
                self.counter = 0
                self.pb.status = 0

    def move_control2(self, cv_image, img_width):
        self.update_match(cv_image, self.scale - 0.1, self.scale + 0.1, self.button_img, 0.7)
        self.pb.move_align(self.button_location[0] + self.button_width * self.press_offset_x, img_width)

    def move_control3(self, cv_image, img_width):
        self.pb.move_range()

    def move_control4(self, cv_image, img_width):
        if self.push_ready:
            self.pb.status = 5
        else:
            rospy.sleep(1)
            self.push_ready = 1
            self.pb.status = 0

    def move_control5(self, cv_image, img_width):
        self.pb.move_torso(0.1)

    def move_control6(self, cv_image, img_width):
        self.pb.push()

    def move_control7(self, cv_image, img_width):
        self.pb.move_torso(0)

    def move_control8(self, cv_image, img_width):
        if self.counter < 100:
            self.counter += 1
        else:
            self.counter = 0
            self.update_match(cv_image, 0.8, 1.5, self.pressed_button_img, 0.9)
            self.pb.status = 9

    def move_control9(self, cv_image, img_width):
        if self.detected:
            self.pb.status = 10
        else:
            self.pb.status = 0

    def move_control10(self, cv_image, img_width):
        self.pb.status = 11
        os.system('roslaunch elevator nav_client.launch point_seq:="[2.25, 1.25, 0]" yaw_seq:="[180]"')
        print("\033[1;32;40m[elevator]: READY\033[0m")


def main(args):
    rospy.init_node('armadillo_elevator_node', anonymous=True, disable_signals=True)

    ic = armadillo_elevator_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
