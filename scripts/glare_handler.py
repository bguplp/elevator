#
# import cv2
# import numpy as np
#
# img = cv2.imread('/home/omri/Desktop/img_test/image4.png')
#
# hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#
# b, g, r = cv2.split(hsv_img)
#
# ret, th = cv2.threshold(b, 127, 255, cv2.THRESH_BINARY)
#
# # output = (b.rows, b.cols, cv2.CV_8UC3)
# #
# # for i in range(b.rows):
# #     for j in range(b.cols):
# #         val = b.at(j, i)
# #         if val == 255:
# #             output.at(j, i)[0] = 189
# #             output.at(j, i)[1] = 108
# #             output.at(j, i)[2] = 47
# #         else:
# #             output.at(j, i)[0] = 94
# #             output.at(j, i)[1] = 206
# #             output.at(j, i)[2] = 236
#
#
# cv2.imshow("hue", th)
# cv2.imshow("b", b)
# cv2.waitKey(0)

# Detection and removal based on study [1].
# Note that notations (r_, m_, s_) are adapted from the paper.
import cv2
import numpy as np
import specularity as spc

impath = '/home/omri/Desktop/img_test/image4.png'
img = cv2.imread(impath)
gray_img = spc.derive_graym(impath)

r_img = m_img = np.array(gray_img)

rimg = spc.derive_m(img, r_img)
s_img = spc.derive_saturation(img, rimg)
spec_mask = spc.check_pixel_specularity(rimg, s_img)
enlarged_spec = spc.enlarge_specularity(spec_mask)

# use opencv's inpaint methods to remove specularity
radius = 12
telea = cv2.inpaint(img, enlarged_spec, radius, cv2.INPAINT_TELEA)
ns = cv2.inpaint(img, enlarged_spec, radius, cv2.INPAINT_NS)