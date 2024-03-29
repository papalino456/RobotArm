import numpy as np
import math
import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.43.181', 12345))

def sendAngles(angles):
    s.sendall(bytes(','.join(map(str, angles)), 'utf-8'))
    #data = s.recv(1024)
    print(angles)
    #print('Received', repr(data.decode('utf-8')))

def calculateJoints(x,y,z,cubeRot=48,bOpen=False):
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

coordList = calculateJoints(200,00,200,48,False)
time.sleep(2)
sendAngles(coordList)

