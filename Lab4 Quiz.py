import sys
import numpy as np
import cv2
import time

# Circles & Hough Transform Function
def test_Circles():
    cap = cv2.VideoCapture(0)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert to hsv color space   
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold of orange to mask using HSV space
        lower_blue = np.array([0, 0, 0]) # Detects red (0)
        upper_blue = np.array([30, 255, 255]) # Detects yellow (30)

        # Creating the mask for overlay
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Masking the raw image
        result = cv2.bitwise_and(frame, frame, mask = mask)

        # Gray the masked image
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        # Blur the masked, gray'd image
        gray = cv2.medianBlur(gray, 5)

        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=50, param2=30, minRadius=1, maxRadius=100)

        # Process each circle calculated from the masked, gray'd image
        # if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(frame, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(frame, center, radius, (255, 0, 255), 3)
            
            # Display the resulting frames
            #cv2.imshow('gray', gray)
            cv2.imshow('mask', mask)
            #cv2.imshow('result', result)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # When everything done, release the capture
            cap.release()
            cv2.destroyAllWindows()
        return center

### PART 1 - Use motor function to spin the robot counterclockwise
from Motor import *            
PWM=Motor()          
def Lab4_CCW():
    print("The robot is spinning counterclockwise")
    while(True):
	    PWM.setMotorModel(-1500,-1500,1500,1500) #modification of left
	    time.sleep(.25)

### PART 2 - Match the robot orientation with the ball's (no velocity)
def Lab4_Orientation():
    print("The robot is following the ball without velocity")
    k = 50
    while(True):
        center = test_Circles()
        pControl = k*(center[0])#X-VALUE OF SCREEN CENTER)
        if pControl <= 0:
            PWM.setMotorModel(pControl,pControl,-pControl,-pControl) #Turn counterclockwise (left)
        else:
            PWM.setMotorModel(-pControl,-pControl,pControl,pControl) #Turn clockwise (richt)
	    time.sleep(.1)

### PART 3 - Match the robot's movements with the ball's
def Lab4_OrientationVelocity():
    print("The robot is following the ball with velocity")
    k = 50
    while(True):
        center = test_Circles()
        pControl = k*(center[0])#X-VALUE OF SCREEN CENTER)
        if pControl <= 0:
            PWM.setMotorModel(pControl,pControl,1.25*-pControl,1.25*-pControl) #Turn counterclockwise (left)
        else:
            PWM.setMotorModel(1.25*-pControl,1.25*-pControl,pControl,pControl) #Turn clockwise (right)
	    time.sleep(.1)