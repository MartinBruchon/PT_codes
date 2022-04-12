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


class ScaleCalibration(Node):                   #create our node class (subclass of Node)

    global caldata
    global scale_real
    global position
    scale_real = 10                        #Real distance in cm to measure for the calibration
    position = []

    
    def __init__(self):                                             #initialisation of the node
        super().__init__('scale')
        self.pub = self.create_publisher(Float32MultiArray, 'calibration_data', 10) #Creation of the publisher. Return an array

        cap = cv2.VideoCapture(2)
        #cap = cv2.VideoCapture("/home/user/Vidéoq/example.mp4")                                 

        while (True):                                           
            success, frame = cap.read()
            cv2.namedWindow('calibration')
            cv2.putText(frame, "Please calibrate with a distance of "+str(scale_real)+" cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
            
            cv2.setMouseCallback('calibration', self.souris)   #call the callback method if there is a click on the window                          


            #show the clicked points on the image
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

                #calculate the distance between the two clicked points
                scale_pix = math.sqrt((XO2 - XO1)**2 + (YO2 - YO1)**2)

                #Store in an array the real length of the QR code's edge and its calculated value in pixel
                caldata = [float(scale_pix), float(scale_real)]

            if cv2.waitKey(1) & 0b1111111 == ord('c'):
                self.publish_data(caldata)
                break
                
            if cv2.waitKey(1) & 0b11111111 == ord('q'):           
                break

            cv2.imshow('calibration', frame)   

        cap.release()                                            
        cv2.destroyAllWindows() 


    def publish_data(self, caldata):       #publish the array containing the real and the calculate length of the QR Code                               
        msg = Float32MultiArray()                                              
        msg.data = caldata                                         
        self.pub.publish(msg) 

    
    def souris(self,event, x, y, flags, param):     #method to stock the coordinates in the image where we clic with the mouse
        
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
    scale = ScaleCalibration()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
