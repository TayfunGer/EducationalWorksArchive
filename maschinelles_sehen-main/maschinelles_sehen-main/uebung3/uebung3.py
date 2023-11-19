import cv2
import numpy as np

Pt1 = 0,0
Pt2 = 0,0
zoom = 1
lut = 1
draw = False
refresh = False
lut_inv = []

# Mouse callback look up Mouse ist Moving look for the left button click
def onMouse(event, x, y, Flags, param):
    global Pt1, Pt2, draw
    if event == cv2.EVENT_MOUSEMOVE:
        if draw:
            Pt2 = (x, y)
    elif event == cv2.EVENT_LBUTTONDOWN:
        draw = True
        Pt1 = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        draw = False

# callback for Zoom refresh the image and apply the Scalefactor
def onZoom(value):
    global zoom, refresh
    zoom = value + 1
    refresh = True

# callback for the lut refresh the image and apply the lut factor
def onLut(value):
    global lut, refresh
    lut = value
    refresh = True

# create LUT array filled with uint8
for i in range(256):
    lut_inv.append(255 - i)
    
lut_inv = np.array(lut_inv).astype('uint8')

lena_gray = cv2.imread('/home/tayfun/dev_ws/maschinelles_sehen/uebung2/Lena.bmp', 0)
lena_gray_rgb = cv2.cvtColor(lena_gray, cv2.COLOR_GRAY2RGB)
tmp_lena = lena_gray_rgb.copy()
cv2. imshow('Lena', lena_gray_rgb)
cv2.moveWindow('Lena', 200, 200)
# Mousecallback on picture Lena
cv2.setMouseCallback('Lena', onMouse)
cv2.createTrackbar('zoom', 'Lena', 0, 3, onZoom)
cv2.createTrackbar('lut', 'Lena', 0, 12, onLut)



# warte bis esc gedrückt ist esc = 27
while (cv2.waitKey(1) != 27):
    if draw or refresh:
        # Define Region of Interest for the zoom
        RoI = lena_gray_rgb[min(Pt1[1], Pt2[1]):max(Pt1[1], Pt2[1]), min(Pt1[0], Pt2[0]):max(Pt1[0], Pt2[0])]
        tmp_lena = lena_gray_rgb.copy()
        cv2.rectangle(tmp_lena, Pt1, Pt2, (0,0,255), 1)
        cv2.imshow('Lena', tmp_lena)
        width = RoI.shape[1] * zoom
        heigth = RoI.shape[0] * zoom
        size = (width, heigth)
        refresh = False
        if width > 0 and heigth > 0:
            zoom_image = cv2.resize(RoI, size)
            if lut < 12:
                zoom_image = cv2.applyColorMap(zoom_image, lut)
            else:
                zoom_image = cv2.LUT(zoom_image, lut_inv)
            cv2.imshow('Lena_zoom_lut', zoom_image)
            cv2.moveWindow('Lena_zoom_lut', 1200,400)
        
cv2.destroyAllWindows()
