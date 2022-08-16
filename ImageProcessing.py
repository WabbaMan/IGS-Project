from ctypes import sizeof
import time
from tkinter import Image
import numpy as np
import cv2
import os
from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
import pyodbc
import sys
import mqttSendPositions as sender
import paho.mqtt.client as mqtt
import time

def takephoto():
    #This creates two variables, the first one 'date' that records the exact date and time when the photo was taken and the second one 'imagePath' that records where the image should be stored to
    date = datetime.now().strftime("%Y_%m_%d-%I-%M-%S_%p")
    imagePath = r'/home/pi/photoFolder/image1.jpg'

    #creates a variable called camera and makes it record what the webcam is displaying
    camera = cv2.VideoCapture(0)

    #Creates a new variable called image and tells it to record what camera has stored in it at that moment
    return_value, image = camera.read()

    del(camera)


    print("Photo Done")
    return image, date

def colourDetection(image):
    ###
    #Uncomment these for testing
    ###
    #filePath = r'/home/pi/photoFolder/testImage.png'
    ###


    #Sets the upper and lower bondaries for the colour detection 
    boundaries = [(([40,40,40]),([70,255,255]))]

    for (lower,upper) in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

    ###########
    #Uncomment for testing
    ###########
    #image = cv2.imread(filePath)
    ###########

    #Changes the colour from the image from RGB to HSV for better colour detection
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    #Creates a mask, by scanning the image for colour that lies outside the range and raplacing it with black, then creates the output by overlaying the mask on the original image
    mask = cv2.inRange(hsv,lower,upper)
    output = cv2.bitwise_and(image, image, mask = mask)
    
    #Calls the next function passing the variable 'date' and 'image' through
    print("Colour detection done")
    return output

def blobDetection(date,output,image):
    #We use simpleBlobDetection from opencv to find the green plants in the photo
    #Creates a new variable,'imagePath' which is where to store the blobDetection photos to for future use
    imagePath = r'/home/pi/photoFolder/'

    #Sets the parameters of the SimpleBlobDectection function
    params = cv2.SimpleBlobDetector_Params()
     
    params.minThreshold = 0
    params.maxThreshold = 100
    params.filterByColor = 0
    params.filterByCircularity = False
    params.filterByArea = True
    params.minArea = 5
    params.maxArea = 5000    
    params.filterByConvexity = False
    params.filterByInertia = False

    #Uses the parameters from above to create a detector then applies that to the image to find the keypoints, the centre of each of the detected blobs
    detector = cv2.SimpleBlobDetector_create(params)
    keyPoints = detector.detect(output)

    #Creates two lists, xCoords and yCoords, the x and y coordinates of the keypoints
    xCoords = []
    yCoords = []

    #Using a for loop, we add the each list by going through the keypoints, adding the y coordinates to yCoords and the x coordinates to xCoords
    for i in range(len(keyPoints)):
        xCoords.append(keyPoints[i].pt[0])
        yCoords.append(keyPoints[i].pt[1])

    XYCoords = []
    for i in range(len(xCoords)):
        XYCoords.append((xCoords[i],yCoords[i]))
    XYTotal = fuse(XYCoords,15)

    im_with_keypoints = cv2.drawKeypoints(output, keyPoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    finalImage = np.hstack([im_with_keypoints,image])
    cv2.imwrite(os.path.join(imagePath, 'opencvBD' + date + '.png'), finalImage)

    cv2.imshow("image", finalImage)
    cv2.waitKey(0)

    fusedXCoords = []
    fusedYCoords = []

    for i in XYTotal:
        fusedXCoords.append(i[0])
        fusedYCoords.append(i[1])

    return fusedXCoords, fusedYCoords

def dist2(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def fuse(points, d):
    ret = []
    d2 = d * d
    n = len(points)
    taken = [False] * n
    for i in range(n):
        if not taken[i]:
            count = 1
            point = [points[i][0], points[i][1]]
            taken[i] = True
            for j in range(i+1, n):
                if dist2(points[i], points[j]) < d2:
                    point[0] += points[j][0]
                    point[1] += points[j][1]
                    count+=1
                    taken[j] = True
            point[0] /= count
            point[1] /= count
            ret.append((point[0], point[1]))
    return ret

def combine(xList, yList):
    # we need to add the sorted points together into a list
    XYTotal = []
    # for each value in both lists
    for i in range(len(xList)):
    # add them to a new list in the format (X,Y)
        XYTotal.append([xList[i],yList[i]])
    return XYTotal

def sizeDetection(xCoords):
    if(len(xCoords) <= 40):
        positions = [[131,142],[175,143],[220,144],[265,145],[309,146],[353,147],[397,148],[454.6,152.8],
                     [135,182],[179,18300],[223,184],[267,185],[311,186],[355,187],[399,188],[444,192.8],
                     [139,222],[183,223],[227,224],[271,225],[315,226],[359,227],[403,228],[447,233],
                     [144,262],[188,263],[232,264],[276,265],[320,266],[364,267],[408,268],[450,273],
                     [148.5,302.2],[192.5,303],[236.5,304],[280.5,305],[324.5,306],[368.5,307],[412.5,308],[447.2,310.5]]
    elif(len(xCoords) <= 104):
        positions = [[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                    ,[000,000],[000,000],[000,000],[000,000]]
    #########
    #for testing
    #positions = [[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]
                #,[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000],[000,000]]
    #########
    return positions

def getPositions(plugXValue, plugYValue, positions):
    positionsStatus = ""
    found = False
    XYTotal = combine(plugXValue, plugYValue)
    XYTotal.sort(key = lambda x: x[0])
    XYTotal.sort(key = lambda x: x[1])

    print("=========================================")
    print("These are the pixel coordinates of each blob")
    print(" ")
    for x in XYTotal:
        print(x)
    print("=========================================") 
    for x in positions:
        for y in XYTotal:
            if (x[0] > y[0] - 22 and x[0] < y[0] + 22) and (x[1] > y[1] - 20 and x[1] < y[0] + 20):
                positionsStatus = positionsStatus + "1,"
                found = True
                break                
        if found == False:
            positionsStatus = positionsStatus + "0,"
        found = False
    return positionsStatus

def startProgram():
    #This function takes no input and returns the photo taken with the camera + the datetime of when the photo was taken
    image, date = takephoto()
    #This function goes over the image, and creates a mask with only the pixels in the colour range showing, it returns the image with the islolated green pixels
    output = colourDetection(image)
    #This function takes the image of isolated pixels and collects nearby pixels to create accurate represenations of where the plants are
    xCoords, yCoords = blobDetection(date, output,image)
    #This function looks at the number of plugs in the tray and returns a size based on that 
    positions = sizeDetection(xCoords)
    #This function compares the coordinates we get from the blobs to the corrdinates 
    positionsStatus = getPositions(xCoords, yCoords, positions)
    sender.sendPositions(positionsStatus)
