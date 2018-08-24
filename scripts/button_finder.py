import cv2
import numpy as np
import sys
import rospkg

class Button_finder:

    def __init__(self, img_rgb):
        self.img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        self.w = -1
        self.h = -1
        self.res = -1

    def find_match_multi_size(self, scale_min, scale_max, temp_img, threshold):
        # detected, \
        # threshold, \
        # button_location, \
        # button_height, \
        # button_width =
        ans = 0, -1, (-1,-1), -1, -1

        template = cv2.imread(temp_img, 0)
        origin_w, origin_h = template.shape[::-1]
        scale = scale_max
        curr_scale = scale_max

        while(scale >= scale_min):
            scaled_template = cv2.resize(template, (int(scale*origin_w),int(scale*origin_h)))
            self.w, self.h = scaled_template.shape[::-1]
            self.res = cv2.matchTemplate(self.img_gray,scaled_template,cv2.TM_CCOEFF_NORMED)
            print("scale {}".format(scale))
            curr_match = self.find_match(0.95, threshold)

            if curr_match[1] == 0.95:
                return scale, curr_match
            elif curr_match[1] > ans[1]:
                ans = curr_match
                curr_scale = scale
            scale -= 0.1

        return curr_scale, ans


    def find_match(self, threshold, min_threshold):
        """
        Args:
            threshold (float): the lower bound of certainty for a match
            res (Map of comparison results): output of cv2.matchTemplate
            w,h (int,int): wanted result size
            img_rgb: given img

        Returns:
            print 'No match was found' if threshold < 0.7
            else: display best estimated match's location with the highest threshold
        """
        if threshold < min_threshold or threshold > 1:
            print("No match was found")
            return 0, -1, (-1, -1), -1, -1

        loc = np.where( self.res >= threshold)
        pts = zip(*loc[::-1])
        if not len(pts):
            if threshold >= 0.7:
                return self.find_match(threshold-0.005, min_threshold)
            else:
                print("No match was found")
                return 0, -1, (-1, -1), -1, -1
        else:
            # print("number of points: {}".format(len(pts)))
            # print("threshold: {}".format(threshold))
            return 1, threshold, pts[0], self.h, self.w
