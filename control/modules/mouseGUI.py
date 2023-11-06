import cv2
import numpy as np
from numpy import linalg as la
import csv
import time
import math as mt
# from kinematics import kine
import matplotlib.path as mpltPath
import os

np.set_printoptions(suppress=True, precision = 2)


# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
# location = "Imperial College London/Imperial/Fluidic Control/ControlSystem/logs"
# location = os.path.dirname(__file__)
# parent = os.path.dirname(location)
# logTime = time.strftime("%Y-%m-%d %H-%M-%S")
# relative = "logs/positions/desired " + logTime + ".csv"
# fileName = os.path.join(parent, relative) # USE THIS IN REAL TESTS
# # fileName = location + "/positions " + logTime + ".csv" 
# # fileName = 'test.csv' # For test purposes
# with open(fileName, mode ='w', newline='') as posLog1: 
#     logger1 = csv.writer(posLog1)
#     logger1.writerow(['Event', 'X', 'Y', 'Z', 'Timestamp', 'ms Since Last', 'xPOI', 'yPOI'])

font = cv2.FONT_HERSHEY_DUPLEX
colourText = (0, 128, 0)
thickText = 1

cableColour = (0, 128, 0)
cableColourLight = (0, 192, 0)
cableThickness = 3

maxCircleColour = (220, 220, 220)
maxThickness = 2

minCircleColour = (192, 192, 192)
minThickness = 2

structColour = (0, 0, 0)
structThickness = 3

handleBoxColour = (0, 0, 0)
handleBoxThickness = 1

colourPOI = (0,0,0)
thicknessPOI = -1

NOMINAL_CANVAS_WIDTH = 750


class mouseTracker:

    def __init__(self,triangleSide, minCable, maxCable):
        # Resolution in mm/pixel (Geomagic touch res ~0.055 mm)
        self.sideLength = triangleSide
        self.canvasX = NOMINAL_CANVAS_WIDTH 
        self.resolution = self.sideLength/self.canvasX
        self.canvasY = int(mt.sqrt(3)*(self.canvasX/2))
        self.centreX = int(self.canvasX/2)
        self.centreY = int(self.canvasY - mt.tan(mt.pi/6)*(self.canvasX/2))
        self.radRestrictPix = self.canvasX*(maxCable/triangleSide)
        self.radRestrPixSma = self.canvasX*(minCable/triangleSide)
        self.mouseEvent = 0
        self.trueMouseEvent = 0
        self.fontscale = 0.5*(self.canvasX/NOMINAL_CANVAS_WIDTH)

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
        # Radius of end effector circle
        self.radius = 5
        # Create an 2num+1 pixel side square around current point
        self.num = 7

        # Give initial values for when class instance is made
        # Cast coordinates as floats for immutability, which allows tracking
        self.xCoord = float(self.centreX/self.canvasX)
        self.yCoord = float((mt.tan(mt.pi/6))*(self.centreX/self.canvasX))
        self.zCoord = 0
        self.xPix = self.centreX
        self.yPix = self.centreY
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



    def drawCables(self, attach_points, POI = None):
        now = time.time()
        touching = False
        insideTri = False
        insideAll = False

        # Draw a 5pixel side square around current point
        p1 = [self.xPix - self.num, self.yPix - self.num]
        p2 = [self.xPix + self.num, self.yPix - self.num]
        p3 = [self.xPix + self.num, self.yPix + self.num]
        p4 = [self.xPix - self.num, self.yPix + self.num]
        neighbour = np.array([p1, p2, p3, p4])
        neighPath = mpltPath.Path(neighbour)
        neighShape = neighbour.reshape((-1, 1, 2))
        # Draw 'handle' box around EE
        cv2.polylines(self.bkGd, [neighShape], True, handleBoxColour, handleBoxThickness)
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
            # if insideAll == True:
            #     if touching == True:
            #         cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (self.xCallback, self.yCallback), cableColourLight , cableThickness)
            #         cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (self.xCallback, self.yCallback), cableColourLight, cableThickness)
            #         cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (self.xCallback, self.yCallback), cableColourLight, cableThickness)

        if POI is not None:
            self.mouseDown = True
            self.touchDown = True
            self.mouseEvent = cv2.EVENT_MOUSEMOVE
            insideAll = True

        if insideAll == True:
            if self.mouseDown == True:
                if self.touchDown == True:
                    # Check if moving
                    if (self.mouseEvent == cv2.EVENT_MOUSEMOVE):
                        # Draw objects:
                        self.drawLines(self.xCallback, self.yCallback, attach_points)
                        # Find values in terms of triangle geometry, not pixels:
                        self.xPix = self.xCallback
                        self.yPix = self.yCallback
                        self.xCoord = self.xPix/self.canvasX
                        yPrime = self.canvasY - self.yPix
                        self.yCoord = yPrime/self.canvasY
                        # Collect data in list to be exported on exit
                        # self.logData.append([self.trueMouseEvent] + [self.desX] + [self.desY] + [self.desZ] + [now] + [self.timeDiff] + \
                        #     [self.xCallback] + [self.yCallback])
        else:
            self.touchDown = False



# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function
    def createTracker(self, attach_points):
        # Create a blank image, a window and bind the callback function to window
        self.drawLines(self.xPix, self.yPix, attach_points)
        # Create a window with a given name
        cv2.namedWindow(self.windowName)
        cv2.moveWindow(self.windowName, 1000, 0)
        # Bind mouseInfo mouse callback function to window
        cv2.setMouseCallback(self.windowName, self.mouseInfo)


    # Redraws the canvas
    def drawLines(self, POI_x, POI_y, attach_points):
        # Reset background
        self.bkGd[:,:] = (255, 255, 255)
        # Draw circles with radius of min reach
        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrPixSma), minCircleColour, minThickness)
        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrPixSma), minCircleColour, minThickness)
        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrPixSma), minCircleColour, minThickness)
        # Draw circles with radius equal to max reach
        cv2.circle(self.bkGd, (self.vt1[0], self.vt1[1]), int(self.radRestrictPix), maxCircleColour, maxThickness)
        cv2.circle(self.bkGd, (self.vt2[0], self.vt2[1]), int(self.radRestrictPix), maxCircleColour, maxThickness)
        cv2.circle(self.bkGd, (self.vt3[0], self.vt3[1]), int(self.radRestrictPix), maxCircleColour, maxThickness)
        # Draw cables in intial position
        # cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (POI_x, POI_y), cableColour, cableThickness)
        # cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (POI_x, POI_y), cableColour, cableThickness)
        # cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (POI_x, POI_y), cableColour, cableThickness)
        # print(attach_points)
        x_lhs = POI_x + round(attach_points[0, 0]/self.resolution)
        y_lhs = POI_y - round(attach_points[1, 0]/self.resolution)

        x_rhs = POI_x + round(attach_points[0, 1]/self.resolution)
        y_rhs = POI_y - round(attach_points[1, 1]/self.resolution)

        x_top = POI_x + round(attach_points[0, 2]/self.resolution)
        y_top = POI_y - round(attach_points[1, 2]/self.resolution)

        cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (x_lhs, y_lhs), cableColour, cableThickness)
        cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (x_rhs, y_rhs), cableColour, cableThickness)
        cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (x_top, y_top), cableColour, cableThickness)
        # Initial position of end effector (25, 14.435)
        cv2.circle(self.bkGd, (POI_x, POI_y), self.radius, colourPOI, thicknessPOI)
        # Entry point triangle
        cv2.polylines(self.bkGd, [self.vts], True, structColour, structThickness)



    def iterateTracker(self, listPress, attach_points, POI = None, desiredPoints = None):
        # Bind mouseInfo mouse callback function to window
        # cv2.setMouseCallback(self.windowName, self.mouseInfo, POI)
        # If path coordinates not specified, use mouse. Path has priority
        if POI is not None:
            # Shift input points so that origin is in bottom corner
            xPosText = -POI[0]
            yPosText = POI[1]
            POI_canvas = POI
            POI_canvas[0] = -POI[0] + self.sideLength/2
            POI_canvas[1] = POI[1] + mt.tan(mt.pi/6)*self.sideLength/2

            self.xCallback = round(POI[0]/self.resolution)
            self.yCallback = self.canvasY - round(POI[1]/self.resolution)

            self.desX = round(desiredPoints[0]/self.resolution)*self.resolution
            self.desY = round(desiredPoints[1]/self.resolution)*self.resolution
            self.desZ = round(desiredPoints[2]/self.resolution)*self.resolution
            self.mouseEvent = cv2.EVENT_MOUSEMOVE

        else:
            xPosText = self.xCoord
            yPosText = self.yCoord

        self.drawCables(attach_points, POI_canvas)
        # if POI is not None:
            # Draw Path points if they are given
            # for i in range(len(self.xPathCoords)):
            #     bkGdX = round(self.xPathCoords[i]/self.resolution)
            #     bkGdY = self.canvasY - round(self.yPathCoords[i]/self.resolution)
            #     if 0 < bkGdX < self.canvasX+1:
            #         if 0 < bkGdY < self.canvasY+1:
            #             self.bkGd[bkGdY, bkGdX] = [0, 0, 255]
                
        # Display pressures:
        P_LHS_Text = "LHS Pressure / mbar = {:.2f}".format(listPress[0])
        P_RHS_Text = "RHS Pressure / mbar = {:.2f}".format(listPress[1])
        P_TOP_Text = "TOP Pressure / mbar = {:.2f}".format(listPress[2])
        pPlaceLHS = (15, 25)
        pPlaceRHS = (15, int(25 + 20*(self.canvasX/NOMINAL_CANVAS_WIDTH)))
        pPlaceTOP = (15, int(25 + 40*(self.canvasX/NOMINAL_CANVAS_WIDTH)))
        pressPlEnd = (int(self.canvasX*0.4), int(self.canvasY/6))
        cv2.rectangle(self.bkGd, (0,0), pressPlEnd, (255,255,255), -1)
        cv2.putText(self.bkGd, P_LHS_Text, pPlaceLHS, font, self.fontscale, colourText, thickText, cv2.LINE_AA)
        cv2.putText(self.bkGd, P_RHS_Text, pPlaceRHS, font, self.fontscale, colourText, thickText, cv2.LINE_AA)
        cv2.putText(self.bkGd, P_TOP_Text, pPlaceTOP, font, self.fontscale, colourText, thickText, cv2.LINE_AA)

        # Redraw position text if moving
        if self.mouseEvent == 0:
            posText = "({:.2f}, {:.2f})".format(xPosText, yPosText)
            placeEE = (self.xPix - int(50*(self.canvasX/NOMINAL_CANVAS_WIDTH)), self.yPix - int(15*(self.canvasX/NOMINAL_CANVAS_WIDTH)))
            placeEERec = (self.xPix - int(50*(self.canvasX/NOMINAL_CANVAS_WIDTH)), self.yPix - int(30*(self.canvasX/NOMINAL_CANVAS_WIDTH))) 
            placeEEEnd = (self.xPix + int(50*(self.canvasX/NOMINAL_CANVAS_WIDTH)), self.yPix - int(12*(self.canvasX/NOMINAL_CANVAS_WIDTH)))
            cv2.rectangle(self.bkGd, placeEERec, placeEEEnd, (255, 255, 255), -1)
            cv2.putText(self.bkGd, posText, placeEE, font, self.fontscale, colourText, thickText, cv2.LINE_AA)
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
        return self.xCoord, self.yCoord, self.stopFlag



    def closeTracker(self):
        # cv2.waitKey = 27
        self.stopFlag = True
        cv2.destroyAllWindows()
        # with open(fileName, 'a', newline='') as posLog:
        #     logger = csv.writer(posLog)
        #     logger.writerows(self.logData)
        return self.stopFlag




if __name__ == "__main__":
    triangleSide = 45
    minCable = 10
    maxCable = 40

    RAD_END = 3.5/2
    ATTACH_POINTS = np.array([[RAD_END*mt.cos(210 * mt.pi/180), RAD_END*mt.sin(210 * mt.pi/180), 0],\
                              [RAD_END*mt.cos(330 * mt.pi/180), RAD_END*mt.sin(330 * mt.pi/180), 0],\
                              [RAD_END*mt.cos(90 * mt.pi/180),  RAD_END*mt.sin(90 * mt.pi/180),  0]])
    ATTACH_POINTS = np.transpose(ATTACH_POINTS)
    # print(ATTACH_POINTS)
    
    ang_around_shaft = 0
    theta_approx = 0

    rotOutOfPlane_yaw = np.array([[mt.cos(ang_around_shaft), -mt.sin(ang_around_shaft), 0],\
                                    [mt.sin(ang_around_shaft),  mt.cos(ang_around_shaft), 0],\
                                    [0,                         0,                        1]])
    
    rotOutOfPlane_pitch = np.array([[ mt.cos(theta_approx), 0, mt.sin(theta_approx)],\
                                    [ 0, 1, 0],\
                                    [-mt.sin(theta_approx), 0, mt.cos(theta_approx)]])
    
    rotOutOfPlane_yawRev = np.array([[mt.cos(-ang_around_shaft), -mt.sin(-ang_around_shaft), 0],\
                                [mt.sin(-ang_around_shaft),  mt.cos(-ang_around_shaft), 0],\
                                [0,                         0,                        1]])
    
    
    attach_points_int0 = np.dot(rotOutOfPlane_yaw, ATTACH_POINTS)
    attach_points_int1 = np.dot(rotOutOfPlane_pitch, attach_points_int0)
    attach_points_rot = np.dot(rotOutOfPlane_yawRev, attach_points_int1)


    mouseTrack = mouseTracker(triangleSide, minCable, maxCable)
    mouseTrack.createTracker(ATTACH_POINTS)
    flagStop = False
    pressL = 0
    pressR = 0
    pressT = 0
    pressList = [pressL, pressR, pressT]
    count = 0
    while flagStop is False:
        [targetX, targetY, flagStop] = mouseTrack.iterateTracker(pressList, attach_points_rot)
        # print(attach_points_rot)
        pressL = 10 + 10*mt.sin(0.01*count)
        pressR = pressL*mt.cos(0.01*count)
        pressT = pressL*mt.sin(0.01*count)
        count = count + 1


    mouseTrack.closeTracker()