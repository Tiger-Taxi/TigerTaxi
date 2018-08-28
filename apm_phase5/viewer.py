import cv2

image = cv2.imread("costmap.png")
cv2.imshow("costmap", image)
cv2.waitKey(0)