#!/usr/bin/env python

from button_finder import Button_finder
import cv2
import math

PXL_PER_RAD = (654.03115, 383.391889)
RAD_PER_PXL = (0.00152898125, 0.0017388641)
INTEL_BUTTON_CENTER = 555

if __name__ == '__main__':
    template = '/home/omri/catkin_ws/src/elevator/img/button_sim.png'
    img = cv2.imread('/home/omri/catkin_ws/src/elevator/img/test1.png')

    img_height, img_width = img.shape[:2]
    # img = img[0:img_height - 150, 0:img_width]
    # img_height, img_width = img.shape[:2]

    bf = Button_finder(img, 0.8)

    scale, \
    (detected, \
     threshold, \
     button_location, \
     button_height, \
     button_width) = bf.find_match_multi_size(0.5, 1, template, 0.5)

    img_height, img_width = img.shape[:2]

    pixel_h = img_height - (button_location[1] + button_height / 2) + (img_height/2 - button_location[1] - button_height / 2)
    pixel_size = 0.025 / button_width
    torso_h = pixel_size * pixel_h
    button_center = button_location[1] + button_height / 2
    torso_h1 = 0.533721 - 0.00116279*button_center
    print("\033[1;32;40mpixel_size = {}\033[0m".format(pixel_size))
    print("\033[1;32;40mtorso_h1 = {}\033[0m".format(torso_h1))

    # X = INTEL_BUTTON_CENTER - (button_location[0] + button_width / 2)
    # pixel_size = 0.025 / button_width
    # press_offset_x = X * pixel_size
    # print("\033[1;32;40mX = {}\033[0m".format(X))
    # print("\033[1;32;40mpress_offset_x = {}\033[0m".format(press_offset_x))
    # theta = RAD_PER_PXL[0] * X
    # dist_y1 = (0.42 - 0.35) * math.tan(theta) * 100
    # print("\033[1;32;40mdist_y = {}\033[0m".format(dist_y1))

    print("\033[1;32;40mbutton_location = ({0}, {1})\033[0m".format(button_location[0] + button_width / 2, button_location[1] + button_height / 2))
    # print("\033[1;32;40mimg_height = {0}, img_width = {1}\033[0m".format(img_height, img_width))
    # print("\033[1;32;40mimg_center = {}\033[0m".format(img_width / 2 - 75))
    font = cv2.FONT_HERSHEY_SIMPLEX
    if detected:
        cv2.putText(img, 'button detected', (img_width - 300, 25), font, 0.8, (0, 153, 0), 2)
        cv2.putText(img, 'certainty: {}%'.format(threshold * 100), (img_width - 300, 50), font, 0.8,
                    (0, 153, 0), 2)
        cv2.putText(img, 'scale: {}'.format(scale), (img_width - 300, 75), font, 0.8, (0, 153, 0), 2)
        cv2.rectangle(img, button_location, (
            button_location[0] + button_width, button_location[1] + button_height),
                      (255, 255, 0), 2)
        cv2.circle(img, (button_location[0] + button_width / 2, button_location[1] + button_height / 2), 5, (255, 255, 255))
        cv2.circle(img, (img_width / 2 - 75, img_height / 2), 5,
                   (0, 0, 255))

    else:
        cv2.putText(img, 'button not detected', (img_width - 300, 25), font, 0.8, (0, 0, 255), 2)

    cv2.imshow("button detector", img)
    cv2.waitKey(0)
