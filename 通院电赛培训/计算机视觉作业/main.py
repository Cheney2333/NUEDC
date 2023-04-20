import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('cat.png')
print(img)


def cv_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

cv_show('image', img)
print(img.shape)
