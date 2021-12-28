import csv
import numpy as np
from numpy import linalg as la
import math as mt
import time
import os


class pathGenerator:
# ALL PATHS MUST START FROM HOME TO BE IN AGREEMENT WITH CALIBRATION ON ARDUINOS
    def __init__(self, triangleSide):

        # Geometry of entry points
        self.sideLength = triangleSide#18.91129205
        self.triHeight = (triangleSide/2)*mt.tan(mt.pi/3)
        self.circCentX = triangleSide/2
        self.circCentY = (triangleSide/2)*mt.tan(mt.pi/6)
        self.circCentZ = 0
        self.xPath = np.array([]) # Poor style but works for now
        self.yPath = np.array([])
        self.zPath = np.array([])

        # File name initialisation
        self.location = os.path.dirname(__file__)
        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "paths/genericPath" + self.logTime + str(self.sideLength) + "EqSide.csv"
        self.fileName = []

    def generatePath(self):

        # Prepend path with slow move to initial position 

        # Convert to lists for improved speed when refereneing individual elements
        self.xPath = self.xPath.tolist()
        self.yPath = self.yPath.tolist()
        self.zPath = self.zPath.tolist()

        with open(self.fileName, mode ='w', newline = '') as pathGenny: 
            pathGenr = csv.writer(pathGenny)
            for j in range(len(self.xPath)):
                pathGenr.writerow([self.xPath[j], self.yPath[j], self.zPath[j]])


    def circlePath(self, numReps):
        noSteps = 180
        circRadius = 0.6250
        circRad = circRadius #np.concatenate([np.linspace(0, self.circRadius, self.noSteps), np.linspace(self.circRadius, 0, self.noSteps)])

        self.relative = "paths/circPath " + self.logTime + " " + str(circRadius) + "mmRad" + str(numReps) + "Reps.csv"
        self.fileName = os.path.join(self.location, self.relative)

        rotStep = np.linspace(0, 2*mt.pi*(1 - 1/(noSteps)), noSteps)
        xPathInter = circRad*np.cos(rotStep) + self.circCentX
        yPathInter = circRad*np.sin(rotStep) + self.circCentY
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)


    def helixPath(self, numReps):
        noSteps = 360
        numRots = 5
        circRadius = 5
        bottomHelix = 5
        topHelix = 35
        circRad = circRadius #np.concatenate([np.linspace(0, self.circRadius, self.noSteps), np.linspace(self.circRadius, 0, self.noSteps)])

        self.relative = "paths/helixPath " + self.logTime + " " + str(circRadius) + "mmRad" + str(numReps) + "Reps.csv"
        self.fileName = os.path.join(self.location, self.relative)

        rotStep = np.linspace(0, numRots*2*mt.pi*(1 - 1/(noSteps)), noSteps)
        xPathInter = circRad*np.cos(rotStep) + self.circCentX
        yPathInter = circRad*np.sin(rotStep) + self.circCentY
        zPathInter = np.linspace(bottomHelix, topHelix, noSteps)
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)
        self.zPath = np.tile(zPathInter, numReps)


    def spiralPath(self, numReps):
        # This is probably wrong - can only do 360 degrees.
        # Better would be Archimedes spiral
        noSteps = 360*2
        circRadius = 1.25
        fwdRadius = np.linspace(0, circRadius, noSteps)
        bwdRadius = np.linspace(circRadius, 0, noSteps)
        spiralRad = np.concatenate((fwdRadius, bwdRadius))

        self.relative = "paths/spiralPath " + self.logTime + " " + str(circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        fwdRot = np.linspace(0, 2*mt.pi*(1 - 1/(noSteps)), noSteps)
        bwdRot = np.linspace(2*mt.pi*(1 - 1/(noSteps)), 0, noSteps)
        rotStep = np.concatenate((fwdRot, bwdRot))

        xPathInter = np.zeros(2*noSteps)
        yPathInter = np.zeros(2*noSteps)
        for i in range(len(spiralRad)):
            xPathInter[i] = spiralRad[i]*np.cos(rotStep[i]) + self.circCentX
            yPathInter[i] = spiralRad[i]*np.sin(rotStep[i]) + self.circCentY
        
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)



    def spiralPath2(self, numReps):
        # Archimedes spiral
        numRots = 5
        noSteps = 180*numRots # number of steps per 
        circRadius = 15
        bottomSpiral = 2.5
        topSpiral = 32.5

        fwdRadius = np.linspace(0, circRadius, noSteps)
        bwdRadius = np.linspace(circRadius, 0, noSteps)
        fwdPrism = np.linspace(bottomSpiral, topSpiral, noSteps)
        bwdPrism = np.linspace(topSpiral, bottomSpiral, noSteps)
        spiralPrism = np.concatenate((fwdPrism, bwdPrism))
        spiralRad = np.concatenate((fwdRadius, bwdRadius))

        self.relative = "paths/spiralZ " + self.logTime + " " + str(circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        fwdRot = np.linspace(0, numRots*2*mt.pi*(1 - 1/(noSteps)), noSteps)
        bwdRot = np.linspace(numRots*2*mt.pi*(1 - 1/(noSteps)), 0, noSteps)
        rotStep = np.concatenate((fwdRot, bwdRot))

        xPathInter = np.zeros(2*noSteps)
        yPathInter = np.zeros(2*noSteps)
        for i in range(len(spiralRad)):
            xPathInter[i] = spiralRad[i]*np.cos(rotStep[i]) + self.circCentX
            yPathInter[i] = spiralRad[i]*np.sin(rotStep[i]) + self.circCentY
        
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)
        self.zPath = np.tile(spiralPrism, numReps)


    def spiralOnCyl(self, numReps):
        # Archimedes spiral on cylinder surface
        numRots = 5
        noSteps = 180*numRots # number of steps per 
        circRadius = 10
        bottomSpiral = 2.5
        topSpiral = 32.5
        cylRad = 10
        flatSpirZ = -5
        spiralCentZ = 20
        spiralCentY = -5
        spiralOffsetY = (cylRad + flatSpirZ) + spiralCentY

        fwdRadius = np.linspace(0, circRadius, noSteps)
        bwdRadius = np.linspace(circRadius, 0, noSteps)
        fwdPrism = np.linspace(bottomSpiral, topSpiral, noSteps)
        # bwdPrism = np.linspace(topSpiral, bottomSpiral, noSteps)
        spiralPrism = flatSpirZ*np.ones(2*noSteps)
        spiralRad = np.concatenate((fwdRadius, bwdRadius))

        self.relative = "paths/spiralOnCyl " + self.logTime + " " + str(circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        fwdRot = np.linspace(0, numRots*2*mt.pi*(1 - 1/(noSteps)), noSteps)
        bwdRot = np.linspace(numRots*2*mt.pi*(1 - 1/(noSteps)), 0, noSteps)
        rotStep = np.concatenate((fwdRot, bwdRot))

        xPathInter = np.zeros(2*noSteps)
        yPathInter = np.zeros(2*noSteps)
        
        for i in range(len(spiralRad)):
            xPathInter[i] = spiralRad[i]*np.cos(rotStep[i])
            yPathInter[i] = spiralRad[i]*np.sin(rotStep[i])
        
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)
        self.zPath = np.tile(spiralPrism, numReps)

        # for i = 1:length(xGUIPath)
        #     Pa(i,:) = dot(Points(i,:), cylAxis(i,:))*cylAxis(i,:);
        #     Pperp(i,:) = Points(i,:) - Pa(i,:);
        #     pDirPerp(i,:) = Pperp(i,:)/norm(Pperp(i,:));
        #     PCyl(i,:) = Pa(i,:) + cylRad*pDirPerp(i,:);
        # end

        vecLen = len(self.xPath)

        cylAxis = np.empty((vecLen, 3))
        Points = np.empty((vecLen, 3))
        Pa = np.empty((vecLen, 3))
        Pperp = np.empty((vecLen, 3))
        pDirPerp = np.empty((vecLen, 3))
        pCyl = np.empty((vecLen, 3))

        cylAxis[:, 0] = np.ones(vecLen)
        cylAxis[:, 1] = np.zeros(vecLen)
        cylAxis[:, 2] = np.zeros(vecLen)
        Points[:, 0] = self.xPath
        Points[:, 1] = self.yPath
        Points[:, 2] = self.zPath

        for i in range(vecLen):
            Pa[i, :] = np.dot(Points[i, :], cylAxis[i, :])*cylAxis[i, :]
            Pperp[i, :] = Points[i, :] - Pa[i, :]
            pDirPerp[i, :] = Pperp[i, :]/la.norm(Pperp[i, :])
            pCyl[i,:] = Pa[i, :] + cylRad*pDirPerp[i, :]

        # self.xPath = np.tile(yPathInter, numReps)
        # self.yPath = np.tile(spiralPrism, numReps)
        # self.zPath = np.tile(xPathInter, numReps)
        self.xPath = pCyl[:, 1] + self.circCentX
        self.yPath = pCyl[:, 2] + self.circCentY + spiralOffsetY
        self.zPath = pCyl[:, 0] + spiralCentZ



    def rasterScan(self, numReps):
        # Eq triangle in circumcircle, radius determined by longest horizontal line (base)
        scanSpeed = 2 #mm/s
        sideFactor = 0.5 # How long base will be wrt side length
        timeStep = 6/125
        maxLenHorLines = sideFactor*self.sideLength
        # Starting point is on line from centre to corner, sideFactor times this length
        heightFactor = 0.05
        lenVerLines = heightFactor*maxLenHorLines
        # Next line up: starts vertically above last, ends at intersection with triangle

        self.relative = "paths/raster " + self.logTime + " " + str(sideFactor) + "B" + str(heightFactor) + "H"\
            + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        # Initialise variables for loop
        currentX = self.circCentX + 0.5*maxLenHorLines
        currentY = self.circCentY - (maxLenHorLines/2)*mt.tan(mt.pi/6)
        xListInter = np.array([])
        yListInter = np.array([])
        # time per line = dist/speed
        # number of 0.048 s steps = time per line/ time per step (steps per line)
        timePerLine = maxLenHorLines/scanSpeed
        stepsPerLine = round(timePerLine/timeStep)
        horDistPerStep = maxLenHorLines/stepsPerLine
        realHorLen = horDistPerStep*stepsPerLine
        # stopPointX = currentX - horDistPerStep*stepsPerLine

        timePerVerLine = lenVerLines/scanSpeed
        stepsPerVerLine = round(timePerVerLine/timeStep)
        verDistPerStep = lenVerLines/stepsPerLine
        realVerLen = verDistPerStep*stepsPerLine
        # stopPointY = currentY + verDistPerStep*stepsPerLine

        lenReduction = 2*lenVerLines/(mt.tan(mt.pi/3))

        lineLen = maxLenHorLines
        direction = 1 # to toggle direction of travel

        # If length of next horizontal line is less than vertical line height, stop and don't make vertical line.
        while lineLen > lenVerLines:
            startPointX = currentX
            startPointY = currentY
            realHorLen = horDistPerStep*stepsPerLine
            stopPointX = currentX - direction*realHorLen
            stopPointY = currentY + realVerLen

            # Horizontal Line coords
            horLineXArray = np.linspace(startPointX, stopPointX, stepsPerLine)
            horLineYArray = np.array(stepsPerLine*[startPointY]) # same y coord repeated stepsPerLine times
            xListInter = np.append(xListInter, horLineXArray)
            yListInter = np.append(yListInter, horLineYArray)

            # Calculate new line length
            lineLen = lineLen - lenReduction
            timePerLine = lineLen/scanSpeed
            stepsPerLine = round(timePerLine/timeStep)
            horDistPerStep = lineLen/stepsPerLine

            # Vertical Line coords
            if lineLen > lenVerLines:
                verLineXArray = np.array(stepsPerVerLine*[stopPointX]) # same x coord repeated stepsPerVerLine times
                verLineYArray = np.linspace(startPointY, stopPointY, stepsPerVerLine)
                xListInter = np.append(xListInter, verLineXArray)
                yListInter = np.append(yListInter, verLineYArray)
            else:
                break

            direction = -1*direction
            currentX = stopPointX
            currentY = stopPointY

        for i in range(0, numReps):
            # If i is odd, add coordinates in order
            if (i % 2 == 1): 
                self.xPath = np.concatenate((self.xPath, xListInter))
                self.yPath = np.concatenate((self.yPath, yListInter))
                # self.yPath.np.append(yListInter)
            # If i is even, add coords in reverse order
            else:
                self.xPath = np.concatenate((self.xPath, np.flip(xListInter)))
                self.yPath = np.concatenate((self.yPath, np.flip(yListInter)))
                # self.xPath.np.append(np.flip(xListInter))
                # self.yPath.np.append(np.flip(yListInter))
        # self.xPath = np.array(xListInter)
        # self.yPath = np.array(yListInter)
     

    def PVMoves(self, numReps):
        """
        Method to move horizontally so that RHS muscle goes from 2 to 17 m contraction with 0.5 mm steps. 
        """
        currentX = 2
        currentY = 0

        minStrain = 2 #mm
        maxStrain = 17 #mm
        stepSize = 1 #mm
        strainRange = maxStrain - minStrain
        numSteps = strainRange/stepSize

        self.relative = "paths/PV " + self.logTime + " 2-17mm " + str(stepSize) + "mm step.csv"
        self.fileName = os.path.join(self.location, self.relative)

        sixSecSamples = 62

        xListInter = np.array(sixSecSamples*[currentX])
        yListInter = np.array(sixSecSamples*[currentY])
        self.xPath = xListInter
        self.yPath = yListInter

        # Create one repetition of steps going from min to max
        for i in range(int(numSteps)):
            # Increment contraction by decreasing Y value
            currentX = currentX + stepSize
            xListInter = np.array(sixSecSamples*[currentX])
            yListInter = np.array(sixSecSamples*[currentY])
            self.xPath = np.concatenate((self.xPath, xListInter))
            self.yPath = np.concatenate((self.yPath, yListInter))

        # Repeat path, varying direction between
        xListInter = self.xPath
        yListInter = self.yPath
        for i in range(0, numReps):
            # If i is odd, add coordinates in order
            if (i % 2 == 1): 
                self.xPath = np.concatenate((self.xPath, xListInter))
                self.yPath = np.concatenate((self.yPath, yListInter))
                # self.yPath.np.append(yListInter)
            # If i is even, add coords in reverse order
            else:
                self.xPath = np.concatenate((self.xPath, np.flip(xListInter)))
                self.yPath = np.concatenate((self.yPath, np.flip(yListInter)))


    def springPos(self, numReps):
        """
        Create path from 2 mm contraction to 17 mm for RHS muscle to follow.
        """
        currentY = 0
        minStrain = 2 #mm
        maxStrain = 17 #mm
        sampsMoving = 150
        sampsPause = 100

        self.relative = "paths/spring " + self.logTime + " 2-17mm " + str(numReps) + "Reps.csv"
        self.fileName = os.path.join(self.location, self.relative)

        np.linspace(minStrain, maxStrain, sampsMoving)

        # Create different parts of the repetition
        xListPauseD = np.array(sampsPause*[minStrain])
        yListPauseD = np.array(sampsPause*[currentY])

        xListUp = np.linspace(minStrain, maxStrain, sampsMoving)
        yListUp = np.array(sampsMoving*[currentY])

        xListPauseU = np.array(sampsPause*[maxStrain])
        yListPauseU = np.array(sampsPause*[currentY])

        xListDown = np.flip(xListUp)
        yListDown = np.array(sampsMoving*[currentY])

        # Put together the parts in order
        xPauseD = np.concatenate((xListPauseD, xListUp))
        yPauseD = np.concatenate((yListPauseD, yListUp))

        xUpPause = np.concatenate((xPauseD, xListPauseU))
        yUpPause = np.concatenate((yPauseD, yListPauseU))

        xOneRep = np.concatenate((xUpPause, xListDown))
        yOneRep = np.concatenate((yUpPause, yListDown))

        problemI = 0
        # Repeat "numReps" number of times
        for i in range(0, numReps):
            self.xPath = np.concatenate((self.xPath, xOneRep))
            self.yPath = np.concatenate((self.yPath, yOneRep))
            problemI = i
        
        # Add a pause at the end
        self.xPath = np.concatenate((self.xPath, xListPauseD))
        self.yPath = np.concatenate((self.yPath, yListPauseD))




    #####
    # Concentric triangles
    #####
        
    # Start at top of triangle
    # Go round triangle 
    # move vertically to next tip

    # numSubTri = 10 # 10 small triangles inside workspace
    # spacing = (self.sideLength/2)*mt.tan(mt.pi/3)/numSubTri # division of triangle height
    # smallestLength = 0.1*self.sideLength
    # startPointX = self.centreX
    # startPointY = self.centreY + spacing
    # # height to centre from bottom left = (triangleSide/2)*mt.tan(mt.pi/6)
    # biggestLength = 0.9*self.sideLength
    # smallestLength = 0.1*self.sideLength


       # Prepend all paths with move from home to start of path
    


sideLength = 18.911 # mm, from workspace2 model
noCycles = 10
pathGen = pathGenerator(sideLength)
pathGen.spiralOnCyl(noCycles)
pathGen.generatePath()