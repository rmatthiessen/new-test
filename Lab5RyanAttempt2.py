import cv2
import cv2.aruco as aruco
import numpy as np
import os
from Motor import *
PWM=Motor()

def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = 10
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs) 
    return [bboxs, ids]
cap = cv2.VideoCapture(0)
try:
    while True:
        success, img = cap.read()
        arucofound = findArucoMarkers(img)
        # loop through all the markers and augment each one
        if  len(arucofound[0])!=0:
            for bbox, id in zip(arucofound[0], arucofound[1]):
                print(bbox)
                sleep = time.sleep(0.001) # Sleep time (self-specified)
                kx = 500 # Gain factor in x
                ky = 500 # Gain factor in y
                centerCameraX = 320 # 1/2 horizontal pixel count of camera
                centerX = int((bbox[0,0,0] + bbox[0,2,0])/2)
                centerY = int((bbox[0,0,1] + bbox[0,2,1])/2)
                sizeX = int((bbox[0,0,0] - bbox[0,2,0]))
                sizeY = abs(int((bbox[0,0,1] - bbox[0,2,1])))
                centerDiffX = int(kx*abs(centerX - centerCameraX))
                sizeDiffY = int(ky*sizeY)
                print("Aruco square pixel distance in X is: ",centerX)
                print("Speed with which to move the robot is: ",centerDiffX)
                print("Pixel height of Aruco is: ",abs(sizeY))
                if centerX < centerCameraX-20: # box is to the left of the screen's center (including a tolerance)
					print("Robot is going left")
					PWM.setMotorModel(-centerDiffX,-centerDiffX,centerDiffX,centerDiffX)
                                        sleep
                                        PWM.setMotorModel(0,0,0,0)
					break
                elif centerX > centerCameraX+20: # box is to the right of the screen's center (including a tolerance)
					print("Robot is going right")
					PWM.setMotorModel(centerDiffX,centerDiffX,-centerDiffX,-centerDiffX)
                                        sleep
                                        PWM.setMotorModel(0,0,0,0)
					break
                else:
                    if sizeY < 50: # box is less than 50 pixels tall
                        PWM.setMotorModel(sizeDiffY,sizeDiffY,sizeDiffY,sizeDiffY)
                        sleep
                        PWM.setMotorModel(0,0,0,0)
                        break
                    elif sizeY > 70: # box is greater than 70 pixels tall
                        PWM.setMotorModel(-sizeDiffY,-sizeDiffY,-sizeDiffY,-sizeDiffY)
                        sleep
                        PWM.setMotorModel(0,0,0,0)
                        break
                    else:
                        PWM.setMotorModel(0,0,0,0)
                #time.sleep(0.05)
        #cv2.imshow('img',img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
except KeyboardInterrupt:
	PWM.setMotorModel(0,0,0,0)
	print('Ctrl-C was pressed.')
