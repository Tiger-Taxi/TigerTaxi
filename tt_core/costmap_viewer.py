import cv2
import os

IMG_DIR = os.path.join(os.environ['TT_ROOT'], 'tt_core', 'img')
COSTMAP_IMG = os.path.join(IMG_DIR, 'costmap.png')

image = cv2.imread(COSTMAP_IMG)
cv2.imshow("costmap", image)
cv2.waitKey(0)
