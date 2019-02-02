#!/usr/bin/env python

import numpy as np
import os
import rospy
import Queue
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
        # get launch params
        self.button_img = rospy.get_param('~panel_img')
        self.pressed_button_img = rospy.get_param('~pressed_button_img')
        self.press_offset_x = float(rospy.get_param('~press_offset_x'))
        self.press_offset_y = float(rospy.get_param('~press_offset_y'))
        self.acc_certainty = float(rospy.get_param('~acc_certainty'))
        self.button_size = float(rospy.get_param('~button_size'))
        self.show_image = rospy.get_param('~show_image')
        self.show_status = rospy.get_param('~show_status')

        self.gripper_pub = rospy.Publisher("/gripper_controller/gripper_cmd/goal", GripperCommandActionGoal,
                                           queue_size=10)
        # for handling movement towards and pushing buttons
        self.pb = push_button()
        self.push_ready = 0
        self.counter = 0
        # dispatching for push_button
        self.move_control = [self.move_control0, self.move_control1, self.move_control2,
                             self.move_control3, self.move_control4, self.move_control5, self.move_control6,
                             self.move_control7, self.move_control8, self.move_control9, self.move_control10]
        self.status_str = ["searching button", "searching button", "align robot to button",
                           "move towards button", "align robot to button", "adjust height to button",
                           "pushing button", "lower robot", "check doors", "failed, trying again",
                           "going into elevator"]

        # move arm to 'button' position
        move_arm.target_move(0, 0, 0, "button")
        rospy.sleep(5)

        # close gripper
        gc = GripperCommandActionGoal()
        gc.goal.command.max_effort = 0.3
        self.gripper_pub.publish(gc)
        rospy.sleep(2)

        self.bridge = CvBridge()
        rospy.Subscriber("/kinect2/qhd/image_color_rect/compressed", CompressedImage, self.kinect_callback)
        rospy.Subscriber("/intel_cam/color/image_raw/compressed", CompressedImage, self.intel_callback)
        # args updated by Button_finder
        self.scale = 1
        self.detected = 0
        self.threshold = -1
        self.button_location = (-1, -1)
        self.button_height = -1
        self.button_width = -1

        self.max_scale = 1
        self.min_scale = 0.2
        self.final_align = 1
        self.align_offset = -30

        self.right_door = 0
        self.left_door = 0

    def intel_callback(self, data):
        # active only for pushing the button
        if self.pb.status != 6:
            return

        try:
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_height, img_width = cv_image.shape[:2]

        if self.show_status:
            print("STATUS: {}".format(self.status_str[self.pb.status]))
        self.move_control[self.pb.status](cv_image, img_width, img_height)

        if self.show_image:
            font = cv2.FONT_HERSHEY_SIMPLEX
            if self.detected:
                cv2.putText(cv_image, 'button detected', (img_width - 300, 25), font, 0.8, (0, 153, 0), 2)
                cv2.putText(cv_image, 'certainty: {}%'.format(self.threshold * 100), (img_width - 300, 50), font, 0.8,
                            (0, 153, 0), 2)
                cv2.putText(cv_image, 'scale: {}'.format(self.scale), (img_width - 300, 75), font, 0.8, (0, 153, 0), 2)
                if self.pb.status != 3:
                    cv2.rectangle(cv_image, self.button_location, (
                        self.button_location[0] + self.button_width, self.button_location[1] + self.button_height),
                                  (255, 255, 0), 2)
            else:
                cv2.putText(cv_image, 'button not detected', (img_width - 300, 25), font, 0.8, (0, 0, 255), 2)
            cv2.imshow("button detector", cv_image)
            cv2.waitKey(3)

    def kinect_callback(self, data):
        # if mission is done, do nothing
        if self.pb.status == 11 or self.pb.status == 6:
            return

        try:
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_height, img_width = cv_image.shape[:2]
        # crop arm from kinect image
        cv_image = cv_image[0:img_height - 150, 0:img_width]
        img_height, img_width = cv_image.shape[:2]

        if self.show_status:
            print("STATUS: {}".format(self.status_str[self.pb.status]))
        self.move_control[self.pb.status](cv_image, img_width, img_height)

        if self.show_image:
            font = cv2.FONT_HERSHEY_SIMPLEX
            if self.detected:
                cv2.putText(cv_image, 'button detected', (img_width - 300, 25), font, 0.8, (0, 153, 0), 2)
                cv2.putText(cv_image, 'certainty: {}%'.format(self.threshold * 100), (img_width - 300, 50), font, 0.8,
                            (0, 153, 0), 2)
                cv2.putText(cv_image, 'scale: {}'.format(self.scale), (img_width - 300, 75), font, 0.8, (0, 153, 0), 2)
                if self.pb.status != 3:
                    cv2.rectangle(cv_image, self.button_location, (
                        self.button_location[0] + self.button_width, self.button_location[1] + self.button_height),
                                  (255, 255, 0), 2)
            else:
                cv2.putText(cv_image, 'button not detected', (img_width - 300, 25), font, 0.8, (0, 0, 255), 2)
            cv2.imshow("button detector", cv_image)
            cv2.waitKey(3)

    def update_match(self, cv_image, scale_min, scale_max, temp_img, threshold):
        """
        Try detecting button and update args accordingly
        """
        bf = Button_finder(cv_image, self.acc_certainty)
        self.scale, \
        (self.detected, \
         self.threshold, \
         self.button_location, \
         self.button_height, \
         self.button_width) = bf.find_match_multi_size(scale_min, scale_max, temp_img, threshold)

    def find_door(self, im):
        q = Queue.Queue()
        q1 = Queue.Queue()
        for i in range(150):
            q.put(0)
            q1.put(0)

        bg = im[:, :, 0] > im[:, :, 1] + 10
        gr = im[:, :, 1] > im[:, :, 2] + 10
        slices = np.bitwise_and(bg, gr, dtype=np.uint8) * 255

        total = 0
        for pixel in slices[80, 0:475]:
            total += (1 if pixel > 0 else 0) - (1 if q.get() > 0 else 0)
            if total >= 130:
                self.left_door = 1
                break
            q.put(pixel)

        total = 0
        for pixel in slices[80, 475:]:
            total += (1 if pixel > 0 else 0) - (1 if q1.get() > 0 else 0)
            if total >= 130:
                self.right_door = 1
                break
            q1.put(pixel)

    ###########################################Dispatching for pushing button###########################################

    def move_control0(self, cv_image, img_width, img_height):
        self.update_match(cv_image, self.min_scale, self.max_scale, self.button_img, 0.6)
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
        if not self.detected:
            self.counter = 0
            self.pb.status = 0
            return
        self.counter += 1
        if self.counter >= 5:
            self.counter = 0
            self.update_match(cv_image, self.scale, self.scale, self.button_img, 0.55)
            self.pb.move_align(self.button_location[0] + self.button_width * self.press_offset_x + self.align_offset,
                               img_width)

    def move_control3(self, cv_image, img_width, img_height):
        self.pb.move_range()
        self.button_img = rospy.get_param('~button_img')
        self.max_scale = 1.5
        self.min_scale = 0.5
        self.align_offset = 0

    def move_control4(self, cv_image, img_width, img_height):
        # final check if aligned
        X = img_width / 2 - (self.button_location[
                                 0] + self.button_width * self.press_offset_x + self.align_offset) - self.align_offset

        # if aligned, lift torso and press
        if 30 >= X >= -30:
            print("\033[1;33m[DEBUG]: aligned, lift torso and press\nX = {}\033[0m".format(X))
            if self.final_align:
                self.update_match(cv_image, self.min_scale, self.max_scale, self.button_img, 0.55)
                self.final_align = 0
            else:
                self.update_match(cv_image, self.scale, self.scale, self.button_img, 0.55)

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
        self.update_match(cv_image, self.scale - 1, self.scale + 1, self.button_img, 0.55)
        button_center = self.button_location[1] + self.button_height / 2
        torso_h = 0.539621 - 0.00116279 * button_center
        print("\033[1;32;40mtorso_h = {}\033[0m".format(torso_h))
        if torso_h > 0.4:
            torso_h = 0.4
        self.pb.move_torso(0.35)

    def move_control6(self, cv_image, img_width, img_height):
        self.button_img = rospy.get_param('~close_button_img')
        self.update_match(cv_image, 0.3, 1.5, self.button_img, 0.6)
        if not self.detected:
            print("\033[1;31mclose button not detected\033[0m")
            self.pb.move_torso(0)
            self.pb.status = 9  # try again
        else:
            x = self.button_location[0] + self.button_width * self.press_offset_x  # + self.align_offset
            pixel_size = self.button_size / self.button_width
            self.pb.push(x, pixel_size)

    def move_control7(self, cv_image, img_width, img_height):
        self.pb.move_torso(0)

    def move_control8(self, cv_image, img_width, img_height):
        # self.pb.move_back(1)
        self.find_door(cv_image)
        if self.left_door:
            print("\033[1;32;40mleft door is open\033[0m")
            self.pb.status += 2
        elif self.right_door:
            print("\033[1;32;40mright door is open\033[0m")
            self.pb.status += 2
        else:
            print("\033[1;31mboth doors are closed, trying again\033[0m")
            self.pb.status += 1

    def move_control9(self, cv_image, img_width, img_height):
        print("\033[1;31mtrying to press again\033[0m")
        rospy.signal_shutdown("Finished Task Successfully")
        #  TODO debugging
        self.pb.move_back(1)
        self.pb.status = 0
        self.push_ready = 0
        self.max_scale = 1.5
        self.min_scale = 0.3
        self.final_align = 1
        self.counter = 0

    def move_control10(self, cv_image, img_width, img_height):
        if self.left_door:
            print("\033[1;32;40mgoing into left elevator\033[0m")
            self.pb.status = 11
            rospy.signal_shutdown("Finished Task Successfully")
            #  TODO debugging
            os.system('roslaunch elevator nav_client.launch point_seq:="[0, 0, 0]" yaw_seq:="[180]"')
        elif self.right_door:
            print("\033[1;32;40mgoing into right elevator\033[0m")
            self.pb.status = 11
            rospy.signal_shutdown("Finished Task Successfully")
            #  TODO debugging
            os.system('roslaunch elevator nav_client.launch point_seq:="[0, 0, 0]" yaw_seq:="[180]"')

    ####################################################################################################################


def main():
    rospy.init_node('armadillo_elevator_node', anonymous=True, disable_signals=True)

    armadillo_elevator_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
