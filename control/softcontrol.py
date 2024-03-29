"""
Read in new position from primary device
Calculate cable lengths at target.
Calcuate volumes at target.
Calculate desired velocity to reach position in given time
    Use Time equal to reciprocal of primary sampling frequency
Calculate rate of cable length change
    Use J and pseudoinverse of J
Calculate rate of volume change (volume flow rate)
Calculate speed of each pump piston
Set step frequency of individual pumps to alter speed
"""

#  Do all imports here or allow them to be made inside modules?
# import serial 

import csv
import traceback
import time
import numpy as np 
# import math
# import random

from modules import arduinoInterface
from modules import kinematics
from modules import mouseGUI
from modules import pumpLog
from modules import streaming
from modules import omniStream


############################################################
# Instantiate classes:
sideLength = 18.911 # mm, from workspace2 model

kineSolve = kinematics.kineSolver(sideLength)
mouseTrack = mouseGUI.mouseTracker(sideLength)
ardLogging = pumpLog.ardLogger()
opTrack = streaming.optiTracker()
phntmOmni = omniStream.omniStreamer()

phntmOmni.connectOmni()

############################################################
pathCounter = 0
cycleCounter = 0

# Count number of reps 
halfCycles = 0
noCycles = 10
antiHystSteps = 50
cDir, targDir = 0, 0
# Use different methods for different paths

xPath = []
yPath = []
zPath = []
# omniX, omniY, omniZ = 0.0, 0.0, 0.0
phntmOmni.getOmniCoords()
[xMap, yMap, zMap] = phntmOmni.omniMap()


# Read path coordinates from file:
with open('paths/spiralOnCyl 2021-04-09 13-01-04 10mmRad18.911EqSide.csv', newline = '') as csvPath:
    coordReader = csv.reader(csvPath)
    for row in coordReader:
        xPath.append(float(row[0]))
        yPath.append(float(row[1]))
        zPath.append(float(row[2]))

# Do you want to use mouse as primary?
useMouse = False

if not useMouse:
    mouseTrack.xCoord = xPath[0]
    mouseTrack.yCoord = yPath[0]
    mouseTrack.zCoord = zPath[0]
    #Down-sample path here for display
    mouseTrack.xPathCoords = xPath[0: int(len(xPath)/noCycles)]  
    mouseTrack.yPathCoords = yPath[0: int(len(yPath)/noCycles)]
    mouseTrack.zPathCoords = zPath[0: int(len(zPath)/noCycles)]


# Initialise variables 
SAMP_FREQ = 1/kineSolve.timeStep
flagStop = False

# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
currentX = mouseTrack.xCoord
currentY = mouseTrack.yCoord
currentZ = mouseTrack.zCoord
targetX = mouseTrack.xCoord
targetY = mouseTrack.yCoord
targetZ = mouseTrack.zCoord

# Create delay at start of any test
delayCount = 0
delayLim = 200
firstMoveDelay = 0
firstMoveDivider = 100
initialXFlag = False
initPressLogCount = 0
initPressLogNum = 10

# Initialise cable length variables at home position
cVolL, cVolR, cVolT, cVolP = 0, 0, 0, 0
cableL, cableR, cableT = kineSolve.sideLength, kineSolve.sideLength, kineSolve.sideLength
prismP = 0
targetP = 0
[targetXideal, targetYideal, targetP] = kineSolve.intersect(targetX, targetY, targetZ)
currentX = targetXideal
currentY = targetYideal
print(targetXideal, targetYideal, targetP)
[targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetXideal, targetYideal)
targetP = 0
print(targetL, targetR, targetT)
repJaco = cJaco
repJpinv = cJpinv

# Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, tStepL, LcRealL, angleL] = kineSolve.length2Vol(cableL, targetL)
[cVolR, tSpeedR, tStepR, LcRealR, angleR] = kineSolve.length2Vol(cableR, targetR)
[cVolT, tSpeedT, tStepT, LcRealT, angleT] = kineSolve.length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
fStepP = 0
tStepP = 0
LcRealP = tStepP/kineSolve.stepsPMM
[LStep, RStep, TStep, PStep] = kineSolve.freqScale(fStepL, fStepR, fStepT, fStepP)
LStep, RStep, TStep, PStep = 0, 0, 0, 0

# Set initial pressure and calibration variables
pressL, pressR, pressT, pressP = 0, 0, 0, 0
prevPressL, prevPressR, prevPressT, prevPressP = 0, 0, 0, 0
pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0
timeL, timeR, timeT, timeP = 0, 0, 0, 0
prevTimeL, prevTimeR, prevTimeT, prevTimeP = 0, 0, 0, 0
conLHS, conRHS, conTOP = 0, 0, 0
dLHS, dRHS, dTOP = 0, 0, 0
collisionAngle = 1j

# Current position
cStepL = tStepL
cStepR = tStepR
cStepT = tStepT
cStepP = tStepP
realStepL, realStepR, realStepT, realStepP = 0, 0, 0, 0
angleL, angleR, angleT, angleP = 0, 0, 0, 0
cRealStepL = realStepL
cRealStepR = realStepR
cRealStepT = realStepT
cRealStepP = realStepP
dStepL, dStepR, dStepT, dStepP  = 0, 0, 0, 0

StepNoL, StepNoR, StepNoT, StepNoP = tStepL, tStepR, tStepT, tStepP
initStepNoL, initStepNoR, initStepNoT = 0, 0, 0
realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA = 0, 0, 0, 0, 0, 0, 0
pneuPress = 1949

###############################################################
# Connect to Arduinos

# Set COM port for each pump
lhsCOM = 8
rhsCOM = 6
topCOM = 7
priCOM = 10
pneuCOM = 15
closeMessage = "Closed"
try:
    ardIntLHS = arduinoInterface.ardInterfacer("LHS", lhsCOM)
    reply = ardIntLHS.connect()
    print(reply)
    ardIntRHS = arduinoInterface.ardInterfacer("RHS", rhsCOM)
    reply = ardIntRHS.connect()
    print(reply)
    ardIntTOP = arduinoInterface.ardInterfacer("TOP", topCOM)
    reply = ardIntTOP.connect()
    print(reply)
    ardIntPRI = arduinoInterface.ardInterfacer("PRI", priCOM)
    reply = ardIntPRI.connect()
    print(reply)
    ardIntPNEU = arduinoInterface.ardInterfacer("PNEU", pneuCOM)
    reply = ardIntPNEU.connect()
    print(reply)

    #############################################################
    # Calibrate arduinos for zero volume - maintain negative pressure for 4 seconds
    calibL = False
    calibR = False
    calibT = False
    calibP = False
    calibA = False
    # Has the mechanism been calibrated/want to run without calibration?:
    calibrated = False
    # Perform calibration:
    while (not calibrated):
        [realStepL, pressL, timeL] = ardIntLHS.listenZero(calibL, pressL, timeL)
        # print(realStepL, pressL)
        [realStepR, pressR, timeR] = ardIntRHS.listenZero(calibR, pressR, timeR)
        # print(realStepR, pressR)
        [realStepT, pressT, timeT] = ardIntTOP.listenZero(calibT, pressT, timeT)
        # print(realStepT, pressT)
        [realStepP, pressP, timeP] = ardIntPRI.listenZero(calibP, pressP, timeP)
        # print(realStepP, calibP)
        [realStepA, pressA, timeA] = ardIntPNEU.listenReply()
        print(pressA, calibA)

        if (realStepL == "0000LHS"):
            calibL = True
        if (realStepR == "0000RHS"):
            calibR = True
        if (realStepT == "0000TOP"):
            calibT = True
        if (realStepP == "0200PRI"):
            calibP = True
        if (pressA >= pneuPress):
            calibA = True
        if (calibL * calibR * calibT * calibP * calibA == 1):
            calibrated = True
            # Send 0s instead of StepNo and pressMed as signal that calibration done
            StepNoL, StepNoR, StepNoT, StepNoP, StepNoA = 0
            pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0

        ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL,\
            realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed,  timeR,\
            realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT,\
            realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP,\
            realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA,\
            conLHS, conRHS, conTOP, collisionAngle)


    ################################################################
    # Begin main loop

    # Bring up GUI
    # mouseTrack.createTracker()
    while(flagStop == False):

        # Stay at first coord for number of cycles
        if delayCount < delayLim:
            delayCount += 1
            pathCounter = 0

        # Go sequentially through path coordinates
        # XYPathCoords = [xPath[pathCounter], yPath[pathCounter]]
        # XYZPathCoords = [xPath[pathCounter], yPath[pathCounter], zPath[pathCounter]]
        phntmOmni.getOmniCoords()
        [xMap, yMap, zMap] = phntmOmni.omniMap()
        XYZPathCoords = [xMap, yMap, zMap]
        # pathCounter += 1

        # Ignore coords from file if mouse is being used
        if useMouse:
            XYZPathCoords = None # Problem here - need to get desired point from GUI but kineSolve.intersect() needs argument first
        # End test when last coords reached
        elif pathCounter >= len(xPath):
            break

        # Ideal target points refer to non-discretised coords on parallel mechanism plane, otherwise, they are discretised.
        # XYZPathCoords are desired coords in 3D.
        [targetXideal, targetYideal, targetP] = kineSolve.intersect(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2])
        [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker(pressL, pressR, pressT,\
            [targetXideal, targetYideal], XYZPathCoords)
        tSecs = tMillis/1000

        # Return target cable lengths at target coords and jacobian at current coords
        [targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetX, targetY)
        tStepP = int(targetP*kineSolve.stepsPMM)
        tStepP += targDir*antiHystSteps
        LcRealP = tStepP/kineSolve.stepsPMM
        # For anti-hysteresis in prismatic joint, check target direction and current direction of motion:
        if tStepP > cStepP:
            targDir = 1
        elif tStepP < cStepP:
            targDir = -1
        # If stopped, preserve previous direction as target: 
        elif tStepP == cStepP:
            targDir = cDir

        # print(targetL, targetR, targetT, LcRealP)
        # Get cable speeds using Jacobian at current point and calculation of input speed
        [lhsV, rhsV, topV, actualX, actualY] = kineSolve.cableSpeeds(currentX, currentY, targetX, targetY, cJaco, cJpinv, tSecs)
        # Find actual target cable lengths based on scaled cable speeds that result in 'actual' coords
        [scaleTargL, scaleTargR, scaleTargT, repJaco, repJpinv] = kineSolve.cableLengths(currentX, currentY, actualX, actualY)
        # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
        [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, scaleTargL)
        [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, scaleTargR)
        [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, scaleTargT)

        # CALCULATE FREQS FROM VALID STEP NUMBER
        # tStepL is target pump position, cStepL is current, speed controlled position.
        fStepL = (tStepL - cStepL)*SAMP_FREQ
        fStepR = (tStepR - cStepR)*SAMP_FREQ
        fStepT = (tStepT - cStepT)*SAMP_FREQ
        fStepP = (tStepP - cStepP)*SAMP_FREQ
        [LStep, RStep, TStep, PStep] = kineSolve.freqScale(fStepL, fStepR, fStepT, fStepP)
        # RStep = dStepR scaled for speed (w rounding differences)
        StepNoL += LStep
        StepNoR += RStep 
        StepNoT += TStep
        StepNoP += PStep

        # Reduce speed when making first move after calibration.
        if firstMoveDelay < firstMoveDivider:
            firstMoveDelay += 1
            # RStep = dStepR scaled for speed (w rounding differences)
            initStepNoL = int(StepNoL*(firstMoveDelay/firstMoveDivider))
            initStepNoR = int(StepNoR*(firstMoveDelay/firstMoveDivider))
            initStepNoT = int(StepNoT*(firstMoveDelay/firstMoveDivider))
            initStepNoP = int(StepNoP*(firstMoveDelay/firstMoveDivider))
            # Send scaled step number to arduinos:
            ardIntLHS.sendStep(initStepNoL)
            ardIntRHS.sendStep(initStepNoR)
            ardIntTOP.sendStep(initStepNoT)
        else:
            # Send step number to arduinos:
            ardIntLHS.sendStep(StepNoL)
            ardIntRHS.sendStep(StepNoR)
            ardIntTOP.sendStep(StepNoT)
        ardIntPRI.sendStep(StepNoP)

        # Calculate median pressure over 10 samples:
        pressLMed = ardIntLHS.newPressMed(pressL)
        pressRMed = ardIntRHS.newPressMed(pressR)
        pressTMed = ardIntTOP.newPressMed(pressT)
        pressAMed = ardIntPNEU.newPressMed(pressA)
        [conLHS, dLHS] = ardIntLHS.derivPress(timeL, prevTimeL)
        [conRHS, dRHS] =  ardIntRHS.derivPress(timeR, prevTimeR)
        [conTOP, dTOP] = ardIntTOP.derivPress(timeT, prevTimeT)
        collisionAngle = kineSolve.collisionAngle(dLHS, dRHS, dTOP, conLHS, conRHS, conTOP)

        # Update current position, cable lengths, and volumes as previous targets
        currentX = actualX
        currentY = actualY
        cableL = scaleTargL
        cableR = scaleTargR
        cableT = scaleTargT
        cVolL = tVolL
        cVolR = tVolR
        cVolT = tVolT
        cStepL = StepNoL
        cStepR = StepNoR
        cStepT = StepNoT
        cStepP = StepNoP
        cDir = targDir
        prevPressL = pressL
        prevPressR = pressR
        prevPressT = pressT
        prevTimeL = timeL
        prevTimeR = timeR
        prevTimeT = timeT

        # Log values from arduinos
        ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL,\
            realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed,  timeR,\
            realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT,\
            realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP,\
            realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA,\
            conLHS, conRHS, conTOP, collisionAngle)

        # Get current pump position, pressure and times from arduinos
        [realStepL, pressL, timeL] = ardIntLHS.listenReply()
        [realStepR, pressR, timeR] = ardIntRHS.listenReply()
        [realStepT, pressT, timeT] = ardIntTOP.listenReply()
        [realStepP, pressP, timeP] = ardIntPRI.listenReply()
        [realStepA, pressA, timeA] = ardIntPNEU.listenReply()

    # Close GUI if Esc hit
    # flagStop = mouseTrack.closeTracker()
    flagStop = False # Will close immediately 



except TypeError as exTE:
    tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
    tb_textTE = ''.join(tb_linesTE)
    print(tb_textTE)

except KeyboardInterrupt as ctrlC:
    print(ctrlC)

    # except Exception as ex:
    #     tb_lines = traceback.format_exception(ex.__class__, ex, ex.__traceback__)
    #     tb_text = ''.join(tb_lines)
    #     print(tb_text)
    

finally:
    ###########################################################################
    # Stop program
    # Disable pumps and set them to idle state
    try:
        # Save values gathered from arduinos
        ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL,\
            realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed,  timeR,\
            realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT,\
            realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP,\
            realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA,\
            conLHS, conRHS, conTOP, collisionAngle)
        ardLogging.ardSave()

        # flagStop = mouseTrack.closeTracker()
        flagStop = True

        if ardIntLHS.ser.is_open:
            ardIntLHS.sendStep(closeMessage)

        if ardIntRHS.ser.is_open:
            ardIntRHS.sendStep(closeMessage)
        
        if ardIntTOP.ser.is_open:
            ardIntTOP.sendStep(closeMessage)

        if ardIntPRI.ser.is_open:
            ardIntPRI.sendStep(closeMessage)

        if ardIntPNEU.ser.is_open:
            ardIntPNEU.sendStep(closeMessage)

        time.sleep(0.2)
        [realStepL, pressL, timeL] = ardIntLHS.listenReply()
        print(realStepL, pressL, timeL)
        time.sleep(0.2)
        [realStepR, pressR, timeR] = ardIntRHS.listenReply()
        print(realStepR, pressR, timeR)
        time.sleep(0.2)
        [realStepT, pressT, timeT] = ardIntTOP.listenReply()
        print(realStepT, pressT, timeT)
        time.sleep(0.2)
        [realStepP, pressP, timeP] = ardIntPRI.listenReply()
        print(realStepP, pressP, timeP)
        time.sleep(0.2)
        [realStepA, pressA, timeA] = ardIntPNEU.listenReply()
        print(realStepA, pressA, timeA)

    except NameError:
        reply = ardIntLHS.connect()
        print(reply)
        reply = ardIntRHS.connect()
        print(reply)
        reply = ardIntTOP.connect()
        print(reply)
        reply = ardIntPRI.connect()
        print(reply)
        reply = ardIntPNEU.connect()
        print(reply)

    except TypeError as exTE:
        tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
        tb_textTE = ''.join(tb_linesTE)
        # print(tb_textTE)

    # Close serial connections
    ardIntLHS.ser.close()
    ardIntRHS.ser.close()
    ardIntTOP.ser.close()
    ardIntPRI.ser.close()
    ardIntPNEU.ser.close()



# if __name__ == '__main__':
#     pass