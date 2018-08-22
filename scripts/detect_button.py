#!/usr/bin/env python

from __future__ import print_function

import roslib
# roslib.load_manifest('elevator')
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

KINECT_YAW = math.pi *62/180
KINECT_PITCH = math.pi *48.6/180

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_buttons",Image, queue_size=10)
    self.gripper_pub = rospy.Publisher("/gripper_controller/gripper_cmd/goal",GripperCommandActionGoal, queue_size=10)

    self.bf = push_button()
    self.mul_check = 1

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/softkinetic/rgb/image_raw",Image,self.camera_callback)
    # self.range = 0

    self.scale = -1
    self.detected = 0
    self.threshold = -1
    self.button_location = (-1,-1)
    self.button_height = -1
    self.button_width = -1

    # move arm to 'button' position
    # move_arm.target_move(0, 0, 0, "button")

    # close gripper
    # gc = GripperCommandActionGoal()
    # gc.goal.command.max_effort = 0.1
    # start_time = rospy.get_time()
    # self.gripper_pub.publish(gc)

  def camera_callback(self,data):
    # if self.range == 0:
    #     return
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img_height, img_width = cv_image.shape[:2]

    # if 0xFF == ord('b'):
    if self.mul_check:
        bf = Button_finder(cv_image)
        self.scale, \
        (self.detected, \
        self.threshold, \
        self.button_location, \
        self.button_height, \
        self.button_width) = bf.find_match_multi_size(scale_min=0.2, scale_max=1)
        self.mul_check = 0
    elif self.bf.status == 0 and self.detected:
        bf = Button_finder(cv_image)
        self.scale, \
        (self.detected, \
        self.threshold, \
        self.button_location, \
        self.button_height, \
        self.button_width) = bf.find_match_multi_size(scale_min=self.scale, scale_max=self.scale)
        self.bf.move[self.bf.status](self.button_location[0], img_width)
    elif self.bf.status > 0:
        self.bf.move[self.bf.status]()


    font = cv2.FONT_HERSHEY_SIMPLEX
    if self.detected:
        cv2.putText(cv_image,'button detected',(img_width-300,25), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'certainty: {}%'.format(self.threshold*100),(img_width-300,50), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'location: {}'.format(self.button_location),(img_width-300,75), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'scale: {}'.format(self.scale),(img_width-300,100), font, 0.8,(0,153,0),2)
        # cv2.putText(cv_image,'range: {}m'.format(self.range),(img_width-300,125), font, 0.8,(0,153,0),2)
        cv2.rectangle(cv_image, self.button_location, (self.button_location[0] + self.button_width, self.button_location[1] + self.button_height), (255,255,0), 2)

        # f_range = float(self.range)
        # W = f_range * math.tan(KINECT_YAW/2) * 2
        # H = f_range * math.tan(KINECT_PITCH/2) * 2
        # X = f_range
        # Y = self.button_location[0]/float(img_width) * W
        # Z = (img_height - self.button_location[1])/float(img_height) * H
        # print("location: ({0},{1},{2})".format(X,Y,Z))

    else:
        cv2.putText(cv_image,'button not detected',(img_width-300,25), font, 0.8,(0,0,255),2)

    cv2.imshow("Head Camera", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)

  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
