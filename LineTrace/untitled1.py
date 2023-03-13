# import the necessary packages

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
#import cv2 as cv
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter
from ipywidgets import interact, IntSlider, Checkbox
import sys
import math
from PIL import Image, ImageDraw

height=200
width=200

image = cv2.imread("face.png")
image = cv2.resize(image, (width,height)) 
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 10)

edged = cv2.Canny(gray, 170, 100)

# find the contours in the edged image
contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
image_copy = image.copy()
# draw the contours on a copy of the original image
cv2.drawContours(image_copy, contours, -1, (0, 255, 0), 1)
print(len(contours), "objects were found in this image.")

imgplot = plt.imshow(gray)
plt.show()

imgplot = plt.imshow(edged)
plt.show()
cv2.waitKey(0)
np_img = np.asarray(edged)
np_img[np_img > 0] = 1
np_img.shape
np_img[height-1]=0

np_img[0]=0
np_img[:,0]=0
np_img[:,width-1]=0
