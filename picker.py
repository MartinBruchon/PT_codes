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
from pyzbar.pyzbar import decode
from std_msgs.msg import Float32MultiArray


class MousePickerCalibration(Node):  

    global position
    position = []                                       

    def __init__(self):                                             #initialisation of the node
        super().__init__('picker')
        
        self.sub = self.create_subscription(Float32MultiArray, 'calibration_data', self.received_msg, 10) #create the subscriber and subscribe to the topic to get the data

    def received_msg(self, caldata):    #if there is a message on the topic...

        qrdata = caldata.data           #stock the array received on the topic
        qr_pix = qrdata[0]              #stock the length in pixel of the QR Code
        qr_real = qrdata[1]             #stock the length in cm of the QR Code
        time.sleep(1)                   #little waiting time to allow the camera to restart correctly
        cap = cv2.VideoCapture(2)  
        #cap = cv2.VideoCapture("/home/user/Vidéoq/example.mp4") 

        while (True):                                           
            success, frame = cap.read()
            cv2.namedWindow('measurement')
            cv2.setMouseCallback('measurement', self.souris)   #callback function if there is a click on the window                           


            #show the clicked points
            if (len(position)==1):
                frame = cv2.circle(frame, (position[0][0],position[0][1]), 3, (255, 0, 0), -1)
            if (len(position)==2):
                
                XO1 = position[0][0]
                YO1 = position[0][1]
                XO2 = position[1][0]
                YO2 = position[1][1]

                frame = cv2.circle(frame, (XO1,YO1), 3, (255, 0, 0), -1)
                frame = cv2.circle(frame, (XO2,YO2), 3, (255, 0, 0), -1)

                frame = cv2.line(frame, (XO1,YO1), (XO2,YO2), (0, 255, 0), 1)

                #calculate the distance in pixel between the two clicked points
                obj_pix = math.sqrt((XO2 - XO1)**2 + (YO2 - YO1)**2)
            
            
                if 'obj_pix' in locals() :
                    obj_real = round(qr_real*obj_pix/qr_pix, 1)    #calculate the real length of the object
                    cv2.putText(frame, str(obj_real) + " cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            cv2.imshow('measurement', frame)                 
            
            
            cv2.waitKey(1)


    #get the coordinates of the clicked points
    def souris(self,event, x, y, flags, param):
        
        if event==cv2.EVENT_LBUTTONDOWN:
            if len(position)==2:
                position.clear()
            position.insert( 0, [x, y])
            #print(position)

        if event==cv2.EVENT_LBUTTONUP:
            position.insert( 1, [x, y])
            #print(position)


def main(args=None):                                        

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    picker = MousePickerCalibration()

    rclpy.spin(picker) #wait here the command to quit. Continue to look for message on topic

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
