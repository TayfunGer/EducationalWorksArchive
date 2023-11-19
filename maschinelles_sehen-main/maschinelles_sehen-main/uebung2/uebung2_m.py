from xml.etree.ElementInclude import include

import cv2

down = False

def onMouse(event, x, y, flags, userdata):
    global pt1, pt2, down, lena_zoom
    if event == cv2.EVENT_LBUTTONDOWN:
        pt1 = (x, y)
        down = True
    elif event == cv2.EVENT_LBUTTONUP:
        pt2 = (x, y)
        lena_tmp = lena_gray_rgb.copy()
        cv2.rectangle(lena_tmp, pt1, pt2, (0, 0, 255))
        cv2.imshow("Lena", lena_tmp)
        lena_zoom = lena_gray_rgb[pt1[1]:pt2[1], pt1[0]:pt2[0]]
        cv2.namedWindow("Zoom", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Zoom", 750, 200)
        cv2.imshow("Zoom", lena_zoom)
        cv2.createTrackbar("Zoom:", "Lena", 0, 3, onChange)
        down = False
    elif event == cv2.EVENT_MOUSEMOVE:
        if down:
            pt2 = (x, y)
            lena_tmp = lena_gray_rgb.copy()
            cv2.rectangle(lena_tmp, pt1, pt2, (0, 0, 255))
            cv2.imshow("Lena", lena_tmp)

def onChange(value):
    global lena_zoom
    height = int(((value * 0.3) + 1) * lena_zoom.shape[0])
    width = int(((value * 0.5) + 1) * lena_zoom.shape[1])
    lena_zoom_tmp = cv2.resize(lena_zoom, (width, height))
    cv2.imshow("Zoom", lena_zoom_tmp)

if __name__ == "__main__":
    lena = cv2.imread("/home/muratcanbaylan/master/maschinelles_sehen/uebung2/Lena.bmp")
    lena_gray = cv2.cvtColor(lena, cv2.COLOR_RGB2GRAY)
    lena_gray_rgb = cv2.cvtColor(lena_gray, cv2.COLOR_GRAY2RGB)
    cv2.namedWindow("Lena")
    cv2.moveWindow("Lena", 200, 200)
    cv2.imshow("Lena", lena_gray_rgb)
    cv2.setMouseCallback("Lena", onMouse)
    cv2.waitKey(100000)