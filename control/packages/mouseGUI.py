import cv2
import numpy as np
from numpy import linalg as la
import csv
import time
import math as mt
# from kinematics import kine
import matplotlib.path as mpltPath
import os



# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
# location = "Imperial College London/Imperial/Fluidic Control/ControlSystem/logs"
location = os.path.dirname(__file__)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/positions " + logTime + ".csv"
fileName = os.path.join(location, relative) # USE THIS IN REAL TESTS
# fileName = location + "/positions " + logTime + ".csv" 
# fileName = 'test.csv' # For test purposes
with open(fileName, mode ='w', newline='') as posLog1: 
    logger1 = csv.writer(posLog1)
    logger1.writerow(['Event', 'X', 'Y', 'Z', 'Timestamp', 'ms Since Last', 'xPOI', 'yPOI'])

font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
colour = (0, 128, 0)
thick = 1

class mouseTracker:

    def __init__(self, triangleSide):
        # Resolution in mm/pixel (Geomagic touch res ~0.055 mm)
        self.resolution = 0.025
        self.sideLength = triangleSide
        self.canvasX = int(self.sideLength/self.resolution)
        self.canvasY = int(mt.sqrt(3)*(self.canvasX/2))
        self.centreX = int(self.canvasX/2)
        self.centreY = int(self.canvasY - mt.tan(mt.pi/6)*(self.canvasX/2))
        self.radRestrictPix = 17/self.resolution #16 mm max contraction
        self.radRestrPixSma = 2.25/self.resolution #4 mm minimum contraction
        self.mouseEvent = 0
        self.trueMouseEvent = 0

        # Create background image
        self.bkGd = np.zeros(( self.canvasY+1, self.canvasX+1, 3), np.uint8)
        self.bkGd[:,:] = (255, 255, 255)
        self.windowName = 'End Effector Tracker'
        # Vertices of equilateral:
        self.vt1 = [0, self.canvasY]
        self.vt2 = [self.canvasX, self.canvasY]
        self.vt3 = [int(self.canvasX/2), 0]
        self.vts = np.array([self.vt1, self.vt2, self.vt3])
        self.path = mpltPath.Path(self.vts)
        self.vts = self.vts.reshape((-1,1,2))
        # Radius of circles that are drawn 
        self.radius = 3
        # Create an 2num+1 pixel side square around current point
        self.num = 10

        # Give initial values for when class instance is made
        # Cast coordinates as floats for immutability, which allows tracking
        self.xCoord = float(self.centreX*self.resolution)#
        self.yCoord = float((mt.tan(mt.pi/6))*(self.centreX*self.resolution))#
        self.zCoord = 0
        self.xPix = int(self.xCoord/self.resolution)#centreX#750#int(self.xCoord/resolution)#
        self.yPix = self.canvasY - int(self.yCoord/self.resolution)#centreY#canvasY-5#
        self.xCallback = self.xPix
        self.yCallback = self.yPix
        self.mouseDown = False
        self.touchDown = False
        self.start = time.time()
        self.timeDiff = 0
        self.prevMillis = 0
        self.stopFlag = False
        self.logData = []
        self.Flags = None
        self.Param = None
        self.xPathCoords = []
        self.yPathCoords = []
        self.zPathCoords = []
        self.desX = []
        self.desY = []
        self.desZ = []



    # Mouse callback function
    def mouseInfo(self, event, x, y, flags, param):
        self.Flags = flags
        self.Param = param
        self.mouseEvent = event
        self.trueMouseEvent = event
        self.xCallback = x
        self.yCallback = y
        if (self.mouseEvent == cv2.EVENT_LBUTTONDOWN) or (param is not None):
            self.mouseDown = True
        if self.mouseEvent == cv2.EVENT_LBUTTONUP:
            self.mouseDown = False



    def drawCables(self, pathCoords = None):
        now = time.time()
        # Draw a 5pixel side square around current point
        p1 = [self.xPix - self.num, self.yPix - self.num]
        p2 = [self.xPix + self.num, self.yPix - self.num]
        p3 = [self.xPix + self.num, self.yPix + self.num]
        p4 = [self.xPix - self.num, self.yPix + self.num]
        neighbour = np.array([p1, p2, p3, p4])
        neighPath = mpltPath.Path(neighbour)
        neighShape = neighbour.reshape((-1, 1, 2))
        # Draw 'handle' box around EE
        cv2.polylines(self.bkGd, [neighShape], True, (0, 0, 0), 1)
        # Check if position is within neighbourhood on down click
        # If inside, move end effector and log.
        touching = neighPath.contains_point([self.xCallback, self.yCallback])
        # Check if point is inside triangle workspace
        insideTri = self.path.contains_point([self.xCallback, self.yCallback])
        radDiff1 = np.array([[self.vt1[0]], [self.vt1[1]]]) - np.array([[self.xCallback], [self.yCallback]])
        radDiff2 = np.array([[self.vt2[0]], [self.vt2[1]]]) - np.array([[self.xCallback], [self.yCallback]])
        radDiff3 = np.array([[self.vt3[0]], [self.vt3[1]]]) - np.array([[self.xCallback], [self.yCallback]])
        proxVt1 = self.radRestrPixSma < la.norm(radDiff1) < self.radRestrictPix
        proxVt2 = self.radRestrPixSma < la.norm(radDiff2) < self.radRestrictPix
        proxVt3 = self.radRestrPixSma < la.norm(radDiff3) < self.radRestrictPix
        insideAll = insideTri*proxVt1*proxVt2*proxVt3
        # print(insideAll)

        # Check if mouse button was pressed down (event = 1)
            # or if params is not None
        if self.mouseDown:
            if touching:
                self.touchDown = True

        # Check if mouse button released (event = 4) and redraw
        if self.mouseEvent == cv2.EVENT_LBUTTONUP:
            self.mouseDown = False
            self.touchDown = False
            # If released inside triangle and within
            # square end effector handle, redraw cables light green
            if insideAll == True:
                if touching == True:
                    cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (self.xCallback, self.yCallback), (0, 192, 0), 1)
                    cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (self.xCallback, self.yCallback), (0, 192, 0), 1)
                    cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (self.xCallback, self.yCallback), (0, 192, 0), 1)

        if pathCoords is not None:
            self.mouseDown = True
            self.touchDown = True
            self.mouseEvent = cv2.EVENT_MOUSEMOVE

        if insideAll == True:
            if self.mouseDown == True:
                if self.touchDown == True:
                    # Check if moving
                    if (self.mouseEvent == cv2.EVENT_MOUSEMOVE):
                        # Reset background
                        self.bkGd[:,:] = (255, 255, 255)
                        # Draw objects:
                        cv2.circle(self.bkGd, (self.xCallback, self.yCallback), self.radius, (0, 0, 0), -1)
                        # Entry point triangle
                        cv2.polylines(self.bkGd, [self.vts], True, (0, 0, 0), 1)
                        # Draw circles with radius of max reach
                        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrictPix), (192, 192, 192), 1)
                        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrictPix), (192, 192, 192), 1)
                        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrictPix), (192, 192, 192), 1)
                        # Draw circles with radius of min reach
                        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
                        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
                        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
                        # Draw cables
                        cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (self.xCallback, self.yCallback), (0, 128, 0), 1)
                        cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (self.xCallback, self.yCallback), (0, 128, 0), 1)
                        cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (self.xCallback, self.yCallback), (0, 128, 0), 1)
                        # Find values in terms of triangle geometry, not pixels:
                        self.xPix = self.xCallback
                        self.yPix = self.yCallback
                        self.xCoord = self.xPix*self.resolution
                        yPrime = self.canvasY - self.yPix
                        self.yCoord = yPrime*self.resolution
                        # Collect data in list to be exported on exit
                        self.logData.append([self.trueMouseEvent] + [self.desX] + [self.desY] + [self.desZ] + [now] + [self.timeDiff] + \
                            [self.xCallback] + [self.yCallback])
        else:
            self.touchDown = False



# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function
    def createTracker(self):
        # Create a blank image, a window and bind the callback function to window
        cv2.polylines(self.bkGd, [self.vts], True, (0, 0, 0), 1)
        # Initial position of end effector (25, 14.435)
        cv2.circle(self.bkGd, (self.xPix, self.yPix), self.radius, (0, 0, 0), -1)
        # Draw circles with radius equal to max reach
        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrictPix), (192, 192, 192), 1)
        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrictPix), (192, 192, 192), 1)
        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrictPix), (192, 192, 192), 1)
        # Draw circles with radius of min reach
        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrPixSma), (192, 192, 192), 1)
        # Draw cables in intial position
        cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        # Create a window with a given name
        cv2.namedWindow(self.windowName)
        # Bind mouseInfo mouse callback function to window
        cv2.setMouseCallback(self.windowName, self.mouseInfo)



    def iterateTracker(self, LHSPress, RHSPress, TOPPress, pathCoords = None, desiredPoints = None):
        # Bind mouseInfo mouse callback function to window
        # cv2.setMouseCallback(self.windowName, self.mouseInfo, pathCoords)
        # If path coordinates not specified, use mouse. Path has priority
        if pathCoords is not None:
            self.xCallback = round(pathCoords[0]/self.resolution)
            self.yCallback = self.canvasY - round(pathCoords[1]/self.resolution)
            self.desX = round(desiredPoints[0]/self.resolution)*self.resolution
            self.desY = round(desiredPoints[1]/self.resolution)*self.resolution
            self.desZ = round(desiredPoints[2]/self.resolution)*self.resolution
            self.mouseEvent = cv2.EVENT_MOUSEMOVE

        self.drawCables(pathCoords)
        # if pathCoords is not None:
            # Draw Path points if they are given
            # for i in range(len(self.xPathCoords)):
            #     bkGdX = round(self.xPathCoords[i]/self.resolution)
            #     bkGdY = self.canvasY - round(self.yPathCoords[i]/self.resolution)
            #     if 0 < bkGdX < self.canvasX+1:
            #         if 0 < bkGdY < self.canvasY+1:
            #             self.bkGd[bkGdY, bkGdX] = [0, 0, 255]
                
        # Display pressures:
        P_LHS_Text = "LHS Pressure / mbar = {:.2f}".format(LHSPress)
        P_RHS_Text = "RHS Pressure / mbar = {:.2f}".format(RHSPress)
        P_TOP_Text = "TOP Pressure / mbar = {:.2f}".format(TOPPress)
        pPlaceLHS = (15,25)
        pPlaceRHS = (15,45)
        pPlaceTOP = (15,65)
        pressPlEnd = (int(self.canvasX*0.4), int(self.canvasY/6))
        cv2.rectangle(self.bkGd, (0,0), pressPlEnd, (255,255,255), -1)
        cv2.putText(self.bkGd, P_LHS_Text, pPlaceLHS, font, fontscale, colour, thick, cv2.LINE_AA)
        cv2.putText(self.bkGd, P_RHS_Text, pPlaceRHS, font, fontscale, colour, thick, cv2.LINE_AA)
        cv2.putText(self.bkGd, P_TOP_Text, pPlaceTOP, font, fontscale, colour, thick, cv2.LINE_AA)

        # Redraw position text if moving
        if self.mouseEvent == 0:
            posText = "({:.2f}, {:.2f})".format(self.xCoord, self.yCoord)
            placeEE = (self.xPix - 60, self.yPix - 15)
            placeEERec = (self.xPix - 60, self.yPix - 30) 
            placeEEEnd = (self.xPix + 60, self.yPix - 12)
            cv2.rectangle(self.bkGd, placeEERec, placeEEEnd, (255, 255, 255), -1)
            cv2.putText(self.bkGd, posText, placeEE, font, fontscale, colour, thick, cv2.LINE_AA)
        # Display image
        cv2.imshow(self.windowName, self.bkGd)
        # Close on Esc key
        if cv2.waitKey(20) & 0xFF == 27:
            self.stopFlag = True
        now = time.time()
        # Save time since beginning code in ms
        numMillis = now - self.start #Still in seconds
        numMillis = numMillis*1000
        self.timeDiff = numMillis - self.prevMillis
        self.prevMillis = numMillis
        return self.xCoord, self.yCoord, self.timeDiff, self.stopFlag



    def closeTracker(self):
        # cv2.waitKey = 27
        self.stopFlag = True
        cv2.destroyAllWindows()
        with open(fileName, 'a', newline='') as posLog:
            logger = csv.writer(posLog)
            logger.writerows(self.logData)
        return self.stopFlag


