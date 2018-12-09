#!/usr/bin/env python

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import queue

im = cv.imread('/home/omri/catkin_ws/src/armadillo2/armadillo2_demos/elevator/img/is_open.png')
im1 = cv.imread('/home/omri/catkin_ws/src/armadillo2/armadillo2_demos/elevator/img/is_open1.png')
im2 = cv.imread('/home/omri/catkin_ws/src/armadillo2/armadillo2_demos/elevator/img/is_open2.png')

OFFSET = 10
q = queue.Queue()
q1 = queue.Queue()

for i in range(150):
    q.put(0)
    q1.put(0)

bg = im[:, :, 0] > im[:, :, 1] + OFFSET  # B == G
gr = im[:, :, 1] > im[:, :, 2] + OFFSET  # G == R
slices = np.bitwise_and(bg, gr, dtype=np.uint8) * 255

bg = im1[:, :, 0] > im1[:, :, 1] + OFFSET  # B == G
gr = im1[:, :, 1] > im1[:, :, 2] + OFFSET  # G == R
slices1 = np.bitwise_and(bg, gr, dtype=np.uint8) * 255

bg = im2[:, :, 0] > im2[:, :, 1] + OFFSET  # B == G
gr = im2[:, :, 1] > im2[:, :, 2] + OFFSET  # G == R
slices2 = np.bitwise_and(bg, gr, dtype=np.uint8) * 255

left_door = 0
right_door = 0

total = 0
for pixel in slices[80, 0:475]:
    total += (1 if pixel > 0 else 0) - (1 if q.get() > 0 else 0)
    if total >= 130:
        left_door = 1
        break
    q.put(pixel)

total = 0
for pixel in slices[80, 475:]:
    total += (1 if pixel > 0 else 0) - (1 if q1.get() > 0 else 0)
    if total >= 130:
        right_door = 1
        break
    q1.put(pixel)

if left_door and right_door:
    print("both doors are open")
elif left_door:
    print("left door is open")
elif right_door:
    print("right door is open")
else:
    print("both doors are closed")

titles = ['test1', 'test2', 'test3',
          'both closed', 'left open', 'right open']
images = [im, im1, im2,
          slices, slices1, slices2]
for i in range(6):
    plt.subplot(2, 3, i + 1), plt.imshow(images[i], 'gray')
    plt.title(titles[i])
    plt.xticks([]), plt.yticks([])
plt.show()

# img = cv.medianBlur(img, 5)
# ret, th1 = cv.threshold(img, 150, 255, cv.THRESH_BINARY)
# th2 = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_MEAN_C, \
#                            cv.THRESH_BINARY, 11, 2)
# th3 = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, \
#                            cv.THRESH_BINARY, 11, 2)
# titles = ['Original Image', 'Global Thresholding (v = 127)',
#           'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
# images = [img, th1, th2, th3]
# for i in range(4):
#     plt.subplot(2, 2, i + 1), plt.imshow(images[i], 'gray')
#     plt.title(titles[i])
#     plt.xticks([]), plt.yticks([])
# plt.show()
