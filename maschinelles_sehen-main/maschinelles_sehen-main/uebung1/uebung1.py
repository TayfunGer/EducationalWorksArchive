from xml.etree.ElementInclude import include


import cv2
import numpy as np 

pic = np.zeros((800,1200,3), np.uint8)

pic[:,:] = 128

#pic = cv2.CV_8U

#cv2.namedWindow("Murat", cv2.WINDOW_FULLSCREEN)
cv2.putText(pic, "test", (600,400), cv2.FONT_HERSHEY_PLAIN, 12, (0, 255, 0), 2, cv2.LINE_AA)
cv2.flip(pic,1, pic)
cv2.namedWindow("Murat", cv2.WINDOW_FULLSCREEN)
cv2.imshow("Murat",pic)


cv2.waitKey(10000)





