from xml.etree.ElementInclude import include

import cv2
import numpy as np 

grey = np.full((400, 600, 3), 128, np.uint8)

cv2.putText(grey, "Test", (300, 200), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 2)
cv2.flip(grey, 1, grey)

cv2.namedWindow("Test")
cv2.moveWindow("Test", 500, 300)
cv2.imshow("Test", grey)

cv2.waitKey(10000)