import sys
import numpy as np
import cv2
import time
from Motor import *            

# Circles & Hough Transform Function
def test_Circles():
    cap = cv2.VideoCapture(0)
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
    center = [];

    # Process each circle calculated from the masked, gray'd image
    if circles is not None:
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
        #cv2.imshow('mask', mask)
        #cv2.imshow('result', result)
        #cv2.imshow('frame', frame)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

    #When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    return center

PWM=Motor()
### PART 1 - Use motor function to spin the robot counterclockwise
def CCW():
    try:
        print("The robot is spinning counterclockwise")
        while(True):
            PWM.setMotorModel(-3000,-3000,3000,3000) #modification of left
            time.sleep(.1)
    except KeyboardInterrupt:
        PWM.setMotorModel(0,0,0,0)
        print ("\nEnd of program")

### PART 2 - Match the robot orientation with the ball's (no velocity)
def Orientation():
    try:
        print("The robot is following the ball without velocity")
        k = 10
        centerX = 320
        while(True):
            center = test_Circles()
            print(center)
            if len(center)!=0:
                pControl = k*(center[0]-centerX)
                if pControl < 0:
                    PWM.setMotorModel(pControl,pControl,-pControl,-pControl) #Turn counterclockwise (left)
                elif pControl > 0:
                    PWM.setMotorModel(-pControl,-pControl,pControl,pControl) #Turn clockwise (richt)
                else:
		    PWM.setMotorModel(0,0,0,0)
    except KeyboardInterrupt:
        PWM.setMotorModel(0,0,0,0)
        print ("\nEnd of program")

### PART 3 - Match the robot's movements with the ball's
def OrientationVelocity():
    try:
        print("The robot is following the ball with velocity")
        k = 50
        while(True):
            center = test_Circles()
            pControl = k*(center[0]-240)
            if pControl <= 0:
                PWM.setMotorModel(pControl,pControl,1.25*-pControl,1.25*-pControl) #Turn counterclockwise (left)
            else:
                PWM.setMotorModel(1.25*-pControl,1.25*-pControl,pControl,pControl) #Turn clockwise (right)
            time.sleep(.1)
    except KeyboardInterrupt:
        PWM.setMotorModel(0,0,0,0)
        print ("\nEnd of program")
        
# Main program logic follows:
if __name__ == '__main__':

    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'test_Circles':
        test_Circles()
    elif sys.argv[1] == 'CCW':
        CCW()
    elif sys.argv[1] == 'Orientation':
        Orientation()
    elif sys.argv[1] == 'OrientationVelocity':
        OrientationVelocity()
