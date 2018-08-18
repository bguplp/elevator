#!/usr/bin/env python

from __future__ import print_function

import roslib
# roslib.load_manifest('elevator')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from button_finder import Button_finder

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_buttons",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect",Image,self.callback)

    self.scale = -1
    self.detected = 0
    self.threshold = -1
    self.button_location = (-1,-1)
    self.button_height = -1
    self.button_width = -1


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    if cv2.waitKey(0) & 0xFF == ord('b'):
        bf = Button_finder(cv_image)
        self.scale, \
        (self.detected, \
        self.threshold, \
        self.button_location, \
        self.button_height, \
        self.button_width) = bf.find_match_multi_size(scale_min=0.2, scale_max=1)

    img_height, img_width = cv_image.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    if self.detected:
        cv2.putText(cv_image,'button detected',(img_width-300,25), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'certainty: {}%'.format(self.threshold*100),(img_width-300,50), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'location: {}'.format(self.button_location),(img_width-300,75), font, 0.8,(0,153,0),2)
        cv2.putText(cv_image,'scale: {}'.format(self.scale),(img_width-300,100), font, 0.8,(0,153,0),2)
        cv2.rectangle(cv_image, self.button_location, (self.button_location[0] + self.button_width, self.button_location[1] + self.button_height), (255,255,0), 2)
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
