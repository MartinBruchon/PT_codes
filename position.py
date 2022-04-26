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
from pyzbar.pyzbar import decode
import math


class ImagePublisher(Node):                                         #create our node class (subclass of Node)

    def __init__(self):                                             #initialisation of the node
        super().__init__('position')
        self.pub = self.create_publisher(Float32MultiArray, 'coordinates', 10)    #create the publisher. The topic will be named qr_topic and will receive string elements
        
        cap = cv2.VideoCapture(2)          #recuperation of the camera flux
        dist_cm = 11.5                    #initialization of the real distance between the centers of both QR Codes (in cm)

        while (True):
            success, frame = cap.read()    #read a frame from the camera
            cv2.namedWindow('original')    #name the window to use it before display

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                          #convert the image in gray
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)                             #add a blur to smooth the images and limit false positives
            thresh = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY)[1]          #create a binary image from the gray scale image
            centers= []
            
            for code in decode(frame):                                      #for all the detected QR Codes :
                                           
                pts = np.array([code.polygon], np.int32)                    #get the points of the angle
                cv2.polylines(frame, [pts], True, (0, 0, 255), 2)           #draw the contours of each QR Code in red

                #get the coordinates of a corner of the QR code
                X1=pts[0][0][0]                
                Y1=pts[0][0][1]


                #get the coordinates of the opposite corner
                X3=pts[0][2][0]
                Y3=pts[0][2][1]

                #calculate the coordinates of the center of the QR Code
                X0 = int((X1 + X3)/2)
                Y0 = int((Y1 + Y3)/2)

                centers.append([X0, Y0])  #store it in an array
                
                if str(code.data.decode('utf-8')) == "origin" :
                    XC = X0
                    YC = Y0
                    frame = cv2.circle(frame, (X0,Y0), 3, (0, 0, 255), -1)   #draw the center of the QR code (red dot)

                if ('centers' in locals()): 

                    if len(centers)==2:
                        theta = math.atan((centers[1][1]-centers[0][1])/(centers[1][0]-centers[0][0]))
                        theta_deg = math.degrees(math.atan(theta))

                        XC2 = int(XC + 1000 * math.cos(theta))
                        YC2 = int(YC + 1000 * math.sin(theta))

                        XC3 = int(XC + 1000 * math.cos(theta + math.pi/2))
                        YC3 = int(YC + 1000 * math.sin(theta +math.pi/2))

                        frame = cv2.line(frame, (XC,YC), (XC2,YC2), (255, 0, 0), 2)
                        frame = cv2.line(frame, (XC,YC), (XC3,YC3), (255, 0, 0), 2)

                        length = math.sqrt((centers[1][1]-centers[0][1])**2 + (centers[1][0]-centers[0][0])**2)

                        mX = (YC2-YC)/(XC2-XC)
                        mY = (YC3-YC)/(XC3-XC+10**-12)

                        bX = YC - mX*XC
                        bY = YC - mY*XC

                contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE) #detect the countours 

                for contour in contours:

                    rect = cv2.minAreaRect(contour)       #get the boundary rectangle of the detected contour (with the minimum area possible)
                    box = np.int0(cv2.boxPoints(rect))    #get the coordinates of this rectangle and store it in an array
                    
                    if cv2.contourArea(contour)> 2000 and cv2.contourArea(contour)< 100000:   #condition on the area of the contour to avoid false positives
                        
                        cv2.drawContours(frame, [box], 0, (0,255,0), 2)    #draw the contour of the object (green rectangle)

                        #store the center coordinates of the detected object (in pixel)
                        x0 = box[0][0]
                        y0 = box[0][1]

                        x1 = box[1][0]
                        y1 = box[1][1]

                        x2 = box[3][0]
                        y2 = box[3][1]

                        edge1 = math.sqrt((x0-x1)**2+(y0-y1)**2) 
                        edge2 = math.sqrt((x0-x2)**2+(y0-y2)**2)

                        if edge1 > edge2:
                            x = x1
                            y = y1
                        else:
                            x = x2
                            y = y2

                        l=100
                        if ('theta' in locals()):
                            xr = int(x + l * math.cos(theta + math.pi/2))
                            yr = int(y + l * math.sin(theta + math.pi/2))
                        
                            frame = cv2.line(frame, (x,y), (xr,yr), (255, 0, 0), 2)

                            frame = cv2.line(frame, (x,y), (x0,y0), (0, 0, 255), 2)

                            rz = math.degrees(math.atan2(yr-y, xr-x) - math.atan2(y0-y, x0-x))
                            if rz<-90:rz+=180
                            if rz>90:rz-=180

                            rz = int(rz)
                            
                        x_obj = int((box[0][0] + box[2][0])/2)
                        y_obj = int((box[0][1] + box[2][1])/2)

                        if ('centers' in locals()): 
                            if len(centers)==2:
                                X2 = (abs(mY*x_obj - y_obj + bY))/(math.sqrt(mY**2 + 1))
                                Y2 = (abs(mX*x_obj - y_obj + bX))/(math.sqrt(mX**2 + 1))
                               
                                X2 = round(X2 * dist_cm / length,1)
                                Y2 = round(Y2 * dist_cm / length,1)

                                if y_obj < mX * x_obj + bX:
                                    Y2 = -1 * abs(Y2)
                               
                                if y_obj < mY * x_obj + bY and mY < 0:
                                    X2 = -1 * abs(X2)
                                if y_obj < mY * x_obj + bY and mY > 0:
                                    X2 = abs(X2)
                                if y_obj > mY * x_obj + bY and mY < 0:
                                    X2 = abs(X2)
                                if y_obj > mY * x_obj + bY and mY > 0:
                                    X2 = -1 * abs(X2)

                                frame = cv2.circle(frame, (x_obj,y_obj), 3, (255, 0, 0), -1)   #draw the center of the object (blue dot)
                                cv2.putText(frame, "( " + str(X2) + " ; " + str(Y2) + " )" + str(rz), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                                
                                coordinates = [X2, Y2, float(rz)]
                                self.publish_data(coordinates)
                    

            if cv2.waitKey(1) & 0b11111111 == ord('q'):
                break
            cv2.imshow('original', frame)

            #cv2.imshow('Gray', gray)
            #cv2.imshow('blurred', blurred)
            #cv2.imshow('thresh', thresh)

        cap.release()
        cv2.destroyAllWindows()                                    #close window

    def publish_data(self, coord):                                 #publish data on the topic
        msg = Float32MultiArray()                                              #the data sent is of type STRING
        msg.data = coord                                           #store the data we want to publish
        self.pub.publish(msg)                                       #publish the data using the publisher

def main(args=None):                                        

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    position = ImagePublisher()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
