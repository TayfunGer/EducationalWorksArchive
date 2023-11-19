import cv2 as cv
import numpy as np

def onTrackbar(val):
    pass


lena_gray = cv.imread('/home/tayfun/dev_ws/maschinelles_sehen/uebung2/Lena.bmp', 0)
cv.imshow('Lena', lena_gray)
cv.moveWindow('Lena', 200, 200)
cv.createTrackbar('Func', 'Lena', 0, 3, onTrackbar)
cv.createTrackbar('Kernel', 'Lena', 0, 14, onTrackbar)
cv.createTrackbar('Sigma', 'Lena', 0, 300, onTrackbar)

while(cv.waitKey(20) != 27):
    func = cv.getTrackbarPos('Func', 'Lena')
    n = cv.getTrackbarPos('Kernel', 'Lena') + 1
    kernel = 2 * n + 1
    sigma = cv.getTrackbarPos('Sigma', 'Lena') / 100
    if func == 0:
        #Mittelwert
       new_image = cv.blur(lena_gray, (kernel, kernel))
    elif func == 1:
        #Gausian
        new_image = cv.GaussianBlur(lena_gray, (kernel, kernel), sigma)
    elif func ==2:
        #Laplacian
        new_image = cv.Laplacian(lena_gray, cv.CV_32F, ksize=int(n / 5) * 2 + 1)
    elif func == 3:
        new_image = lena_gray - cv.GaussianBlur(lena_gray, (kernel, kernel), sigma) + 127
    else: 
        print("Failure")
        new_image = lena_gray
    #new_image = np.uint8(np.clip((alpha * lena_gray + beta), 0, 255))
    cv.imshow("Lena Filter", new_image)
    cv.moveWindow("Lena Filter", 1200, 400)

cv.destroyAllWindows()

