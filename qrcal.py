"""
Technical Project
BRUCHON Martin
"""

#import the different librairies and modules we use

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import math
from pyzbar.pyzbar import decode


class QRCodeCalibration(Node):                                         #create our node class (subclass of Node)

    global caldata
    global qr_real
    qr_real = 5        #real length of the edge of the QR Code (in cm)
    
    def __init__(self):                                             #initialisation of the node
        super().__init__('qrcal')
        self.pub = self.create_publisher(Float32MultiArray, 'calibration_data', 10) #Creation of the publisher. Return an array

        cap = cv2.VideoCapture(2)                                        #recuperation of the camera flux   
        #cap = cv2.VideoCapture("/home/user/Vidéoq/example.mp4")

        while (True):                                           
            success, frame = cap.read()                                  #read a frame from the camera
            cv2.namedWindow('calibration')                               #name the window to use it before display
            
            for code in decode(frame):                                   #for all the detected QR Codes :            
                pts = np.array([code.polygon], np.int32)                 #get the points of the angle
                cv2.polylines(frame, [pts], True, (0, 0, 255), 2)        #draw the contours of each QR Code in red
                
                #get the coordinates of a corner of the QR code
                X1=pts[0][0][0]
                Y1=pts[0][0][1]

                #get the coordinates of a corner next to the first one
                X2=pts[0][1][0]
                Y2=pts[0][1][1]

                qr_pix = math.sqrt((X2 - X1)**2 + (Y2 - Y1)**2)           #calculation of the length of an edge of the QR Code (in pixel)
                caldata = [float(qr_pix), float(qr_real)]                               #Store in an array the real length of the QR code's edge and its calculated value in pixel

                #draw used points of the QR Code
                frame = cv2.circle(frame, (X1,Y1), 3, (255, 0, 0), -1)
                frame = cv2.circle(frame, (X2,Y2), 3, (255, 0, 0), -1)

            if cv2.waitKey(1) & 0b1111111 == ord('c'):                   #call the function to publish data when 'c' is pressed
                self.publish_data(caldata)
                break
                
            if cv2.waitKey(1) & 0b11111111 == ord('q'):           
                break

            cv2.imshow('calibration', frame)   

        cap.release()                                            
        cv2.destroyAllWindows() 

    def publish_data(self, caldata):          #publish the array containing the real and the calculate length of the QR Code                               
        msg = Float32MultiArray()                                              
        msg.data = caldata                                         
        self.pub.publish(msg) 
    

def main(args=None):                                        

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    qrcal = QRCodeCalibration()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
