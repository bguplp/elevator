#!/usr/bin/env python

import roslib
import sys
import math
import rospy
import cv2
from sensor_msgs.msg import Image
from control_msgs.msg import GripperCommandActionGoal
from cv_bridge import CvBridge, CvBridgeError
from button_finder import Button_finder
from push_button import push_button
import move_arm

class image_converter:

  def __init__(self):
    self.button_img = rospy.get_param('~button_img')
    self.pressed_button_img = rospy.get_param('~pressed_button_img')
    self.press_offset_x = float(rospy.get_param('~press_offset_x'))
    self.press_offset_y = float(rospy.get_param('~press_offset_y'))

    self.image_pub = rospy.Publisher("image_buttons",Image, queue_size=10)
    self.gripper_pub = rospy.Publisher("/gripper_controller/gripper_cmd/goal",GripperCommandActionGoal, queue_size=10)

    self.pb = push_button()
    self.mul_check = 1
    self.push_ready = 0

    # move arm to 'button' position
    move_arm.target_move(0, 0, 0, "button")
    rospy.sleep(3)

    # close gripper
    gc = GripperCommandActionGoal()
    gc.goal.command.max_effort = 0.1
    self.gripper_pub.publish(gc)
    rospy.sleep(2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/softkinetic/rgb/image_raw",Image,self.camera_callback)

    self.scale = -1
    self.detected = 0
    self.threshold = -1
    self.button_location = (-1,-1)
    self.button_height = -1
    self.button_width = -1

  def camera_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img_height, img_width = cv_image.shape[:2]

    if self.mul_check:
        self.update_match(cv_image, 0.2, 1.8, self.button_img, 0.7)
        self.mul_check = 0

    elif self.pb.status in range(0,4) and self.detected:
        if self.pb.status == 0:
            self.update_match(cv_image, self.scale, self.scale, self.button_img, 0.7)
        elif self.pb.status == 2 and not self.push_ready:
            self.update_match(cv_image, 0.8, 1.8, self.button_img, 0.7)
            self.push_ready = 1
            self.pb.status = 0
        self.pb.move[self.pb.status](self.button_location[0] + self.button_width * self.press_offset_x, img_width, 0.1)

    elif self.pb.status == 4:
        self.pb.move_torso(0, 0, 0)
        self.update_match(cv_image, 0.6, 1.8, self.pressed_button_img, 0.8)
        if self.detected:
            move_arm.target_move(0, 0, 0, "driving")
            self.pb.status = 5
            rospy.signal_shutdown("Pressed button, finished task successfully")
        else:
            self.pb.status = 0
            self.mul_check = 0
            self.push_ready = 0

    font = cv2.FONT_HERSHEY_SIMPLEX
    if self.detected:
        cv2.putText(cv_image,'button detected',(img_width-300,25), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'certainty: {}%'.format(self.threshold*100),(img_width-300,50), font, 0.8,(0,153,0),2)
        # cv2.putText(cv_image,'location: {}'.format(self.button_location),(img_width-300,75), font, 0.8,(0,153,0),2)
        # cv2.putText(cv_image,'scale: {}'.format(self.scale),(img_width-300,100), font, 0.8,(0,153,0),2)
        if self.pb.status != 1:
            cv2.rectangle(cv_image, self.button_location, (self.button_location[0] + self.button_width, self.button_location[1] + self.button_height), (255,255,0), 2)

    else:
        cv2.putText(cv_image,'button not detected',(img_width-300,25), font, 0.8,(0,0,255),2)

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


def main(args):
  rospy.init_node('image_converter', anonymous=True, disable_signals=True)

  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
