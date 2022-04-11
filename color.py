"""
Technical Project
BRUCHON Martin
"""

import cv2
import numpy as np
 
def souris(event, x, y, flags, param):                  #mouse action
    
    if event==cv2.EVENT_LBUTTONDBLCLK:                  #on double left click...
        color=image[y, x][0]                            #...take the color of the pixel
    lo[0]=color-5                                     #create a range of color
    hi[0]=color+5

global lo, hi, color
color=100
lo=np.array([color-5, 100, 50])
hi=np.array([color+5, 255,255])
color_info=(0, 0, 255)

cap=cv2.VideoCapture(2)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris) 
                 #mouse listener
while True:                                             #loop
    ret, frame=cap.read()
    image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        #convert the image from BGR (= RGB) to HSV which is another color code (easiest for our application)
    image=cv2.blur(image, (5, 5))                       #add a little blur to be smoother and limit false positives 
    mask=cv2.inRange(image, lo, hi)                     #create a mask to isolate a selected color range (it's a binary image)
    mask=cv2.erode(mask, None, iterations=2)            
    mask=cv2.dilate(mask, None, iterations=2)
    image2=cv2.bitwise_and(frame, frame, mask= mask)    #logical AND to overlay the original image and the mask
  
    cv2.imshow('Camera', frame)                         #show the original image
    cv2.imshow('image2', image2)                        #show the color only
    #cv2.imshow('Mask', mask)
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

