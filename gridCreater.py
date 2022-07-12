import numpy as np
import cv2
import os
from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
import pyodbc

def takephoto(camNumber):

    #This creates two variables, the first one 'date' that records the exact date and time when the photo was taken and the second one 'imagePath' that records where the image should be stored to
    date = datetime.now().strftime("%Y_%m_%d-%I-%M-%S_%p")
    imagePath = r'C:\Users\JoeGilligan\IGS_Project\IGS-Project\photoFolder'

    #creates a variable called camera and makes it record what the webcam is displaying
    camera = cv2.VideoCapture(camNumber)

    #Creates a new variable called image and tells it to record what camera has stored in it at that moment
    return_value, image = camera.read()

    del(camera)
        
    #Calls the next function, colourDetection, passing the variables through to that function
    print("Photo Done")
    return image, date

def colourDetection(image):
    #Current filePath only there for testing, later will use image from takePhoto
    filePath = r'C:\Users\JoeGilligan\IGS_Project\IGS-Project\photoFolder\testImage.png' 
    
    #Sets the upper and lower bondaries for the colour detection 
    boundaries = [(([40,40,40]),([70,255,255]))]

    for (lower,upper) in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

    #Changes the colour from the image from RGB to HSV for better colour detection
    #Currently reads a test image from the drive, in future this will be the image passed from takePhoto
    image = cv2.imread(filePath)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #Creates a mask, by scanning the image for colour that lies outside the range and raplacing it with black, then creates the output by overlaying the mask on the original image
    mask = cv2.inRange(hsv,lower,upper)
    output = cv2.bitwise_and(image, image, mask = mask)
    
    #Calls the next function passing the variable 'date' and 'image' through
    print("Colour detection done")
    return output, image

def blobDetection(date,image,output):

    #We use simpleBlobDetection from opencv to find the green plants in the photo
    #Creates a new variable,'imagePath' which is where to store the blobDetection photos to for future use
    imagePath = r'C:\Users\JoeGilligan\IGS_Project\IGS-Project\photoFolder'

    #Sets the parameters of the SimpleBlobDectection function
    params = cv2.SimpleBlobDetector_Params()
     
    params.minThreshold = 0
    params.maxThreshold = 100
    params.filterByColor = 0
    params.filterByCircularity = False
    params.filterByArea = True
    params.minArea = 50
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
    XYTotal = fuse(XYCoords,50)
        
    #Creates a new image called im_with_keypoints which is the original image with the keypoints superimposed on top of it and then saves this image to the imagePath
    im_with_keypoints = cv2.drawKeypoints(output, keyPoints, np.array([]), (255,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    finalImage = np.hstack([im_with_keypoints,image])
    cv2.imwrite(os.path.join(imagePath, 'opencvBD' + date + '.png'), finalImage)
        
    #Displays im_with_keypoints in a new window
    cv2.imshow("Keypoints", finalImage)
    cv2.waitKey(0)

    print("Blob detection done")
    return xCoords, yCoords

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

def convertCoords(xCoords, yCoords, size, camNumber):
    # There are 3 different sizes of tray, and each size has its own parameters for how to plugs are aligned
    # The formatting for the alignment is poaint in range (min, max, value point is changed to)
    if size == 0 :
        xList = [(0, 150, 130), (150, 250, 230), (250, 350, 330), (350, 450, 430), (450, 550, 530), (550, 650, 630), (650, 750, 730), (750, 850, 830)]
        yList = [(0, 175, 150), (175, 275, 230), (275, 350, 310), (350, 450, 400), (450, 525, 480), (525, 600, 575), (600, 700, 650), (700, 775, 732.5), (775, 860, 830), (860, 950, 900), (950, 1050, 1000), (1050, 1150, 1090), (1150, 1200, 1175), (1200, 1300, 1270), (1300, 1400, 1350), (1400, 1500, 1450), (1500, 1600, 1550), (1600, 1700, 1640), (1700, 1750, 1725), (1750, 1850, 1800)]
    elif size == 1:
        xList = [()]
        yList = [()]
    elif size == 2:
        xList = [()]
        yList = [()]
    
    # call function sort with the list of plug points and the list of how to format those points
    xSorted = sort(xCoords, xList)
    ySorted = sort(yCoords, yList)

    if camNumber == 0:
        # we need to add the sorted points together into a list
        XYTotal = []
        # for each value in both lists
        for i in range(len(xSorted)):
        # add them to a new list in the format (X,Y)
            XYTotal.append((xSorted[i],ySorted[i]))
        XYTotal = list(dict.fromkeys(XYTotal))
        return XYTotal
    else:
        return xSorted, ySorted
# 
def sort(coords, items):
    result = []
    for coord in coords:
        for item in items:
            if item[0] < coord < item[1]:
                result.append(item[2])
    return result

def emptyPlugFinder(XYTotal, size):
    emptyPlugX = []
    emptyPlugY = []
    if size == 0 :
        xList = [130, 230, 330, 430, 530, 630, 730, 830]
        yList = [150, 230, 310, 400, 480, 575, 650, 732.5, 830, 900, 1000, 1090, 1175, 1270, 1350, 1450, 1550, 1640, 1725, 1800]
    elif size == 1:
        xList = []
        yList = []
    elif size == 2:
        xList = []
        yList = []
    found = False
    for i in xList:
        for s in yList:    
            checkingPoint = [i, s]
            for o in XYTotal:
                if o == (checkingPoint[0],checkingPoint[1]):
                    found = True
                    break
            if found == False:
                emptyPlugX.append(checkingPoint[0])
                emptyPlugY.append(checkingPoint[1])
            found = False
    print("Number of failed plugs: " + str(len(emptyPlugX)))
    return emptyPlugX, emptyPlugY

def sendData(plugXValue, plugYValue):
    conn = pyodbc.connect("DSN=TestConnection;UID=SYSDBA;PWD=masterkey")
    cursor = conn.cursor()
    for n in range(len(plugXValue)):
        cursor.execute("INSERT INTO badPlugTable(badPlugX, badPlugY) VALUES (?, ?)", plugXValue[n], plugYValue[n])
    conn.commit()

def startProgram(camNumber, size):
    image, date = takephoto(camNumber)
    output, image = colourDetection(image)
    xCoords, yCoords = blobDetection(date, image, output)
    if camNumber == 0:
        XYTotal = convertCoords(xCoords, yCoords, size, camNumber)
        plugXValue, plugYValue = emptyPlugFinder(XYTotal, size)
    else:
        plugXValue, plugYValue = convertCoords(xCoords, yCoords, size, camNumber)
    sendData(plugXValue, plugYValue)


startProgram(0, 0)
