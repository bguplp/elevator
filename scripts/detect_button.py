#!/usr/bin/env python

import sys
import numpy as np
import os
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from control_msgs.msg import GripperCommandActionGoal
from cv_bridge import CvBridge, CvBridgeError
from button_finder import Button_finder
from push_button import push_button
import move_arm


class armadillo_elevator_node:
    """
    This node is responsible for handling the elevator mission with Armadillo2 robot.
    """

    def __init__(self):
        # get outer button images and offsets
        self.button_img = rospy.get_param('~button_img')
        self.pressed_button_img = rospy.get_param('~pressed_button_img')
        self.press_offset_x = float(rospy.get_param('~press_offset_x'))
        self.press_offset_y = float(rospy.get_param('~press_offset_y'))

        # # for publishing camera view with image processing
        # self.image_pub = rospy.Publisher("image_buttons", Image, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper_controller/gripper_cmd/goal", GripperCommandActionGoal,
                                           queue_size=10)
        # for handling movement towards and pushing buttons
        self.pb = push_button()
        self.push_ready = 0
        self.counter = 0
        self.inside_elevator = 0
        # dispatching for push_button
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
        gc.goal.command.max_effort = 0.2
        self.gripper_pub.publish(gc)
        rospy.sleep(2)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color/compressed", CompressedImage, self.camera_callback)

        # args updated by Button_finder
        self.scale = -1
        self.detected = 0
        self.threshold = -1
        self.button_location = (-1, -1)
        self.button_height = -1
        self.button_width = -1

        self.max_scale = 3
        self.min_scale = 0.2
        self.final_align = 1

    def camera_callback(self, data):
        # if mission is done, do nothing
        if self.pb.status == 11:
            return

        try:
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_height, img_width = cv_image.shape[:2]
        # crop arm from kinect image
        cv_image = cv_image[0:img_height - 150, 0:img_width - 100]
        img_height, img_width = cv_image.shape[:2]

        print("STATUS: {}".format(self.status_str[self.pb.status]))
        self.move_control[self.pb.status](cv_image, img_width, img_height)

        # font = cv2.FONT_HERSHEY_SIMPLEX
        # if self.detected:
        #     cv2.putText(cv_image, 'button detected', (img_width - 300, 25), font, 0.8, (0, 153, 0), 2)
        #     cv2.putText(cv_image, 'certainty: {}%'.format(self.threshold * 100), (img_width - 300, 50), font, 0.8,
        #                 (0, 153, 0), 2)
        #     cv2.putText(cv_image, 'scale: {}'.format(self.scale), (img_width - 300, 75), font, 0.8, (0, 153, 0), 2)
        #     # cv2.circle(cv_image, (img_width/2-100, img_height-100), 5, (255, 255, 255))
        #     if self.pb.status != 3:
        #         cv2.rectangle(cv_image, self.button_location, (
        #             self.button_location[0] + self.button_width, self.button_location[1] + self.button_height),
        #                       (255, 255, 0), 2)

        # else:
        #     cv2.putText(cv_image, 'button not detected', (img_width - 300, 25), font, 0.8, (0, 0, 255), 2)

        # cv2.imshow("button detector", cv_image)
        # cv2.waitKey(3)

        # try:
        # publish the processed image
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)

    def update_match(self, cv_image, scale_min, scale_max, temp_img, threshold):
        """
        Try detecting button and update args accordingly
        """
        bf = Button_finder(cv_image)
        self.scale, \
        (self.detected, \
         self.threshold, \
         self.button_location, \
         self.button_height, \
         self.button_width) = bf.find_match_multi_size(scale_min, scale_max, temp_img, threshold)

    ###########################################Dispatching for pushing button###########################################

    # def move_control_debug(self, cv_image, img_width, img_height):
    #     self.counter += 1
    #     if self.counter > 100:
    #         self.pb.status = 1
    #         self.counter = 0
    #         print("update_match")
    #     return

    def move_control0(self, cv_image, img_width, img_height):
        self.update_match(cv_image, self.min_scale, self.max_scale, self.button_img, 0.7)
        self.pb.status = 1

    def move_control1(self, cv_image, img_width, img_height):
        if self.detected:
            self.pb.status = 2
        else:
            self.counter += 1
            self.pb.move_align(-10, img_width)
            if self.counter >= 30:
                self.counter = 0
                self.pb.status = 0

    def move_control2(self, cv_image, img_width, img_height):
        self.counter += 1
        if self.counter >= 5:
            self.counter = 0
            self.update_match(cv_image, self.scale, self.scale, self.button_img, 0.7)
            self.pb.move_align(self.button_location[0] + self.button_width * self.press_offset_x, img_width)

    def move_control3(self, cv_image, img_width, img_height):
        self.pb.move_range()

    def move_control4(self, cv_image, img_width, img_height):
        # final check if aligned
        X = img_width / 2 - (self.button_location[0] + self.button_width * self.press_offset_x) - 30

        # if aligned, lift torso and press
        if 15 >= X >= -15:
            print("\033[1;33m[DEBUG]: aligned, lift torso and press\nX = {}\033[0m".format(X))
            if self.final_align:
                self.update_match(cv_image, self.min_scale, self.max_scale, self.button_img, 0.7)
                self.final_align = 0
            else:
                self.update_match(cv_image, self.scale, self.scale, self.button_img, 0.7)

            self.counter += 1
            if self.counter >= 10:
                self.counter = 0
                self.pb.status = 5
        # else align again
        else:
            print("\033[1;31m[DEBUG]: align again\nX = {}\033[0m".format(X))

            self.counter = 0
            self.pb.status = 2

    def move_control5(self, cv_image, img_width, img_height):
        torso_h = (1 - (self.button_location[1] + self.button_height / 2) / float(img_height))
        print("torso height = {}".format(torso_h))
        self.pb.move_torso(torso_h)

    def move_control6(self, cv_image, img_width, img_height):
        # self.max_scale += 1
        self.pb.push(self.button_location[0] + self.button_width * self.press_offset_x, img_width)

    def move_control7(self, cv_image, img_width, img_height):
        # debugging - stop here
        self.pb.move_torso(0)
        self.pb.status = 11
        print("\033[1;32;40mDONE\033[0m")
        rospy.signal_shutdown("Finished Task Successfully")

    def move_control8(self, cv_image, img_width, img_height):
        if self.counter < 100:
            self.counter += 1
        else:
            self.counter = 0
            self.update_match(cv_image, self.min_scale, self.max_scale, self.pressed_button_img, 0.9)
            self.pb.status = 9

    def move_control9(self, cv_image, img_width, img_height):
        if self.detected:
            self.pb.status = 10
        else:
            self.pb.status = 0

    def move_control10(self, cv_image, img_width, img_height):

        # if self.inside_elevator == 2:
        #     self.counter += 1
        #     if self.counter > 100:
        #         self.inside_elevator = 1
        #         self.counter = 0
        #         self.pb.status = 11

        if self.inside_elevator:
            os.system('roslaunch elevator nav_client.launch point_seq:="[0, 0, 0]" yaw_seq:="[180]"')
            print("\033[1;32;40mFinished Task Successfully\033[0m")
            rospy.signal_shutdown("Finished Task Successfully")
            return

        # get inside the elevator
        os.system('roslaunch elevator nav_client.launch point_seq:="[0.5, 0, 0, 2.25, 1.25, 0]" yaw_seq:="[0, 135]"')
        # move arm to 'button' position
        move_arm.target_move(0, 0, 0, "button")
        # push inner button
        self.button_img = rospy.get_param('~inner_button_img')
        self.pressed_button_img = rospy.get_param('~pressed_inner_button_img')
        # reset args
        self.pb.status = 0
        self.push_ready = 0
        self.max_scale = 3
        self.min_scale = 0.2
        self.final_align = 1
        rospy.sleep(8)
        # wait before detecting
        self.inside_elevator = 2

    ####################################################################################################################


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
