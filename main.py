import socket
import cv2
import numpy as np
import math

def sendAngles(angles):
    s.sendall(bytes(','.join(map(str, angles)), 'utf-8'))
    data = s.recv(1024)
    print(angles)
    print('Received', repr(data.decode('utf-8')))

def calculateJoints(x,y,z):
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
        scaling_factor = max_radius / actual_distance
        x *= scaling_factor
        y *= scaling_factor
        z *= scaling_factor
    theta1 = math.degrees(math.atan2(y,x))
    theta2 = math.degrees(math.atan2(z,x) + math.acos((A3**2+(x**2+z**2)-(A4+A5)**2)/(2*A3*(math.sqrt(x**2+z**2)))))
    theta3 = -(math.degrees(math.acos((A3**2+(A4+A5)**2-(math.sqrt(x**2+z**2))**2)/(2*A3*(A4+A5))) - math.pi))
    joints = [theta1, theta2, theta3, 0, 0]
    return joints

def processImage(frame):
    coordList = []
    image = frame

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blur = cv2.medianBlur(gray, 5)

    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
    
    ret, thresh = cv2.threshold(sharpen, 0, 255, cv2.THRESH_OTSU)
    print("Used threshold value: ", ret)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    min_area = 300
    max_area = 1500
    image_number = 0  

    cv2.drawContours(image, contours, -1, (0, 0, 255), 3)
    cv2.circle(image, (0, 0), 5, (255, 0, 0), -1)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area and area < max_area:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(image, [box], 0, (36,255,12), 2)
            coordList.append(rect[0])
            cv2.circle(frame, (int(rect[0][0]), int(rect[0][1])), 5, (255, 0, 0), -1)
    print(coordList)
    return coordList


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.100.121', 12345))

#cap = cv2.VideoCapture(0)

#ret, frame = cap.read()
#if ret:
sendAngles(calculateJoints(0,0,500))
#cv2.imshow('Video Stream', frame)
#cv2.waitKey(0)
#cap.release()
