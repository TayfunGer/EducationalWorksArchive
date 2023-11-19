import cv2 as cv
import numpy as np

def onTrackbar(val):
    pass


lena_gray = cv.imread('/home/tayfun/dev_ws/maschinelles_sehen/uebung2/Lena.bmp', 0)
cv.imshow('Lena', lena_gray)
cv.moveWindow('Lena', 200, 200)
cv.createTrackbar('brightness', 'Lena', 0, 255, onTrackbar)
cv.createTrackbar('contrast', 'Lena', 0, 300, onTrackbar)

while(cv.waitKey(500) != 27):
    beta = cv.getTrackbarPos('brightness', 'Lena') - 127
    alpha = cv.getTrackbarPos('contrast', 'Lena') / 100
    new_image = np.uint8(np.clip((alpha * lena_gray + beta), 0, 255))
    cv.imshow('Lena cont brig', new_image)
    cv.moveWindow('Lena cont brig', 1200, 400)

cv.destroyAllWindows()

