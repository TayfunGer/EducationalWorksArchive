import cv2
import numpy as np 

#def onMouse(event, x, y, flags, param):
#    match event:
#        case cv2.EVENT_MOUSEMOVE:
            

grey_image = cv2.imread("Lena.bmp", 0)

gray_in_color = cv2.cvtColor(grey_image, cv2.COLOR_GRAY2BGR)

tmp_image = grey_image.copy()
#image_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#cv2.namedWindow("LENA GREY")


cv2.imshow("LENA GREY", tmp_image)

#cv2.copyTo(image, cv2.CV_8U,image_cpy)

cv2.waitKey(10000)

cv2.destroyAllWindows()

