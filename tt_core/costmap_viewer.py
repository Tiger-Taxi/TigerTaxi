import cv2
import os

image = cv2.imread(os.path.join("img", "costmap.png"))
cv2.imshow("costmap", image)
cv2.waitKey(0)
