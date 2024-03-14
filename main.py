import socket
import cv2
import numpy as np
import math
import time


def sendAngles(angles):
    s.sendall(bytes(','.join(map(str, angles)), 'utf-8'))
    data = s.recv(1024)
    print(angles)
    print('Received', repr(data.decode('utf-8')))


def calculateJoints(x,y,z,cubeRot=0,bOpen=False):
    A1=55.0
    A2=30.0
    A3=200.0
    A4=200.0
    A5=90.0
    # Calculate the radius of the semi-sphere domain of the robot
    max_radius = A3 + A4 + A5
    # Calculate the actual distance of the point from the origin
    actual_distance = math.sqrt(x**2 + y**2 + z**2)
    # Clamp the x, y, z values if they are outside the semi-sphere domain
    if actual_distance > max_radius:
        print("Point outside range, clamping x, y, z values")

    cosTheta3 = max(min(((x**2+y**2+z**2-A3**2-(A4+A5)**2)/(2*A3*(A4+A5))), 1), -1)
    senTheta3 = -math.sqrt(1-cosTheta3**2)
    theta1 = math.degrees(math.atan2(y,x))
    #theta2 = math.degrees(math.atan2(z,x) + math.acos((A3**2+(x**2+z**2)-(A4+A5)**2)/(2*A3*(math.sqrt(x**2+z**2))))) #OLD theta 
    theta2 = math.degrees(math.atan2(z,math.sqrt(x**2+y**2))-math.atan2((A4+A5)*senTheta3,A3+(A4+A5)*cosTheta3))
    #theta3 = -(math.degrees(math.acos((A3**2+(A4+A5)**2-(math.sqrt(x**2+z**2))**2)/(2*A3*(A4+A5))) - math.pi)) #OLD theta
    theta3 = -(math.degrees((math.atan2(senTheta3,cosTheta3))))
    #print(theta1, theta2, theta3)
    openVal = 70 if bOpen else 0
    joints = [theta1, theta2, theta3, cubeRot, openVal]
    return joints

def findZero(frame):
    # Convert image to HSV color space
    image = frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    ret, thresh = cv2.threshold(sharpen, 0, 255, cv2.THRESH_OTSU)
    print("Used threshold value: ", ret)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    cv2.imshow('close', close)
    circles = cv2.HoughCircles(close, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=20, minRadius=10, maxRadius=30)
    
    # If circles are found, draw them and set pixelDiameter to the diameter of the first found circle
    if circles is not None:
        circles = np.uint16(np.around(circles))
        pixelDiameter = circles[0][0][2] * 2  # Diameter is twice the radius
        print("Pixel Diameter: ", pixelDiameter)
        cv2.circle(frame, (circles[0][0][0], circles[0][0][1]), 5, (255, 0, 255), -1)
        cv2.circle(frame, (circles[0][0][0], circles[0][0][1]), 2, (0, 0, 255), 3)
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        zero = (circles[0][0][0], circles[0][0][1])
        cv2.imshow('circles', frame)
    else:
        zero = None
        pixelDiameter = None  # Ensure pixelDiameter is None if no circles are found
    return zero, pixelDiameter

def processImage(frame):
    coordList = []
    #image process
    image = frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    ret, thresh = cv2.threshold(sharpen, 0, 255, cv2.THRESH_OTSU)
    print("Used threshold value: ", ret)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    cv2.drawContours(image, contours, -1, (0, 0, 255), 3)
    cv2.circle(frame, pixelZero, 5, (255, 0, 255), -1)
    min_area = 400
    max_area = 2500
    image_number = 0  

    #square detection
    for cnt in contours:
        area = cv2.contourArea(cnt)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        width = rect[1][0]
        height = rect[1][1]
        if area > min_area and area < max_area and (width/height > 0.75 and width/height < 1.25):  # Check if shape is almost square
            cv2.drawContours(image, [box], 0, (36,255,12), 2)
            coordList.append(rect[0])
            cv2.circle(frame, (int(rect[0][0]), int(rect[0][1])), 5, (255, 0, 0), -1)
    cv2.imshow('Video Stream', frame)
    
    return frame, coordList

def pixelToMM(coordList, pixelDiameter):
    mmList = []
    for (x, y) in coordList:
        # Convert pixel coordinates to mm from pixelZero
        x_mm_from_zero = -((x + pixelZero[0]) * 15) / pixelDiameter
        y_mm_from_zero = ((y + pixelZero[1]) * 15) / pixelDiameter
        # Translate coordinates to coincide with RealMMZeroOffset
        RealX = x_mm_from_zero + RealMMZeroOffset[0]
        RealY = y_mm_from_zero + RealMMZeroOffset[1]
        mmList.append((RealX, RealY))
    print(mmList)

    return mmList

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.43.181', 12345))

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


ret, frame = cap.read()
if ret:
    pixelZero, pixelDiameter = findZero(frame)
    RealMMZeroOffset = (270, 30)  # in mm
    image, pixelList = processImage(frame)
    
    coordList = pixelToMM(pixelList, pixelDiameter)
    print(coordList)
    if coordList:
        sendAngles(calculateJoints(coordList[0][0], coordList[0][1], 0,48,True))
        time.sleep(2)
        sendAngles(calculateJoints(coordList[0][0], coordList[0][1], 0,48,False))
        time.sleep(2)
        sendAngles(calculateJoints(coordList[0][0], coordList[0][1], 100,48,False))
    else:
        print("No object found")
cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()
