import cv2
import os

image = cv2.imread("costmap.png")
cv2.imshow(os.path.join("img", "costmap"), image)
cv2.waitKey(0)
