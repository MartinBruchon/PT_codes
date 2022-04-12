"""
Technical Project
BRUCHON Martin
"""

#import the different librairies and modules we use

import time
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from std_msgs.msg import Float32MultiArray


class Shape(Node):

    def __init__(self):
        
        super().__init__('shape')
        self.sub = self.create_subscription(Float32MultiArray, 'calibration_data', self.received_msg, 10)    #create the subscriber and subscribe to the topic to get the data
        
    def received_msg(self, caldata):       #if there is a message on the topic...
        qrdata = caldata.data              #stock the array received on the topic
        qr_pix = qrdata[0]                 #stock the length in pixel of the QR Code
        qr_real = qrdata[1]                #stock the length in cm of the QR Code
        time.sleep(1)                      #little waiting time to allow the camera to restart correctly
        cap = cv2.VideoCapture(2)
        #cap = cv2.VideoCapture("/home/user/Vidéoq/example.mp4")
        cv2.namedWindow("camera")
        cv2.createTrackbar( "sensitiviity", "camera", 0, 255, self.sensitivity)    #create a trackbar to adjust the sensitivity if needed

        while (True):

            
            if ('sensitivity' in globals()):
                seuil = sensitivity
            else : 
                seuil = 60
            
            
            success, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                          #convert the image in gray
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)                             #add a blur to smooth the images and limit false positives
            thresh = cv2.threshold(blurred, seuil, 255, cv2.THRESH_BINARY)[1]          #create a binary image from the gray scale image

            contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE) #detect the countours 

            for contour in contours:

                rect = cv2.minAreaRect(contour)        #get the boundary rectangle of the detected contour (with the minimum area possible)
                box = np.int0(cv2.boxPoints(rect))     #get the coordinates of this rectangle and store it in an array
                
                if cv2.contourArea(contour)> 200 and cv2.contourArea(contour)< 100000:         #condition on the area of the contour to avoid false positives

                    cv2.drawContours(frame, [box], 0, (36,255,12), 2)                          #draw the contour of the object (green rectangle)

                    object_X = math.sqrt((box[0][0]-box[1][0])**2+(box[0][1]-box[1][1])**2)    #get the X length of the object in pixel
                    object_Y = math.sqrt((box[0][0]-box[3][0])**2+(box[0][1]-box[3][1])**2)    #get the X length of the object in pixel
                    
                    
                    obj_real_X = round(qr_real*object_X/qr_pix, 1)   #convert it in cm
                    obj_real_Y =round(qr_real*object_Y/qr_pix, 1)    #convert it in cm

                    cv2.putText(frame, str(obj_real_X) + " cm x " + str(obj_real_Y) +" cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
            #cv2.imshow('thresh', thresh)  
            cv2.imshow('camera', frame)

            #cv2.imshow('Gray', gray)
            #cv2.imshow('blurred', blurred)
            
            cv2.waitKey(1)

        

    def sensitivity(self,*arg): #change the sensitivity of the according to the position of the trackbar
        global sensitivity
        sensitivity = arg[0]
        

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    shape = Shape()

    rclpy.spin(shape)

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
