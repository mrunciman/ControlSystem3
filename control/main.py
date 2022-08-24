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
# from modules import mouseGUI
from modules import pumpLog
from modules import positionInput
from modules import optiStream
from modules import omniStream
from modules import energyShaping

# from visual_navigation.cam_pose import PoseEstimator

np.set_printoptions(suppress=True, precision = 2)
############################################################
# Instantiate classes:
sideLength = 30 # mm, from workspace2 model

kineSolve = kinematics.kineSolver(sideLength)
# mouseTrack = mouseGUI.mouseTracker(sideLength)
ardLogging = pumpLog.ardLogger()
posLogging = positionInput.posLogger()
opTrack = optiStream.optiTracker()
phntmOmni = omniStream.omniStreamer()
energyS = energyShaping.energyShaper()


############################################################
pathCounter = 0
prevPathCounter = 0
keyboardIn = 0
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

omni_connected = phntmOmni.connectOmni()

# Try to connect to phantom omni. If not connected, use pre-determined coords.
if not omni_connected:
    with open('control/paths/gridPath 2022-07-11 11-36-55 centre 15-8.66025 30x30grid 5x5spacing 110pris.csv', newline = '') as csvPath:
        coordReader = csv.reader(csvPath)
        for row in coordReader:
            xPath.append(float(row[0]))
            yPath.append(float(row[1]))
            zPath.append(float(row[2]))
        xMap, yMap, zMap = xPath[0], yPath[0], zPath[0]
else:
    # omniX, omniY, omniZ = 0.0, 0.0, 0.0
    phntmOmni.getOmniCoords()
    [xMap, yMap, zMap] = phntmOmni.omniMap()

# xPath[0], yPath[0], zPath[0] = 15, 8.66, 20
# xMap, yMap, zMap = xPath[1155], yPath[1155], zPath[1155]+10
XYZPathCoords = [xMap, yMap, zMap]
print("Start point: ", XYZPathCoords)

# Do you want to use mouse as primary?
# useMouse = False

# if not useMouse:
#     mouseTrack.xCoord = xPath[0]
#     mouseTrack.yCoord = yPath[0]
#     mouseTrack.zCoord = zPath[0]
#     #Down-sample path here for display
#     mouseTrack.xPathCoords = xPath[0: int(len(xPath)/noCycles)]  
#     mouseTrack.yPathCoords = yPath[0: int(len(yPath)/noCycles)]
#     mouseTrack.zPathCoords = zPath[0: int(len(zPath)/noCycles)]


############################################################################
# Initialise variables 
SAMP_FREQ = 1/kineSolve.TIMESTEP
PRESS_MAX_PA = 160000
PRESS_MIN_PA = 70000
CLOSEMESSAGE = "Closed"
flagStop = False

# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
currentX = XYZPathCoords[0]
currentY = XYZPathCoords[1]
currentZ = XYZPathCoords[2]
targetX = XYZPathCoords[0]
targetY = XYZPathCoords[1]
targetZ = XYZPathCoords[2]

# Create delay at start of any test
delayCount = 0
delayLim = 200
delayEveryStep = False
delayFactor = 8
firstMoveDelay = 0
firstMoveDivider = 100
initialXFlag = False
initPressLogCount = 0
initPressLogNum = 10
useVisionFeedback = False
visionFeedFlag = False
useEnergyShaping = True
energyFlag = False

# desired positon for energy shaping
x_d = 0.00#5
firstflag = True
controlInputs = [0, 0, 0]
pos_x, vel_x = 0, 0

# Initialise cable length variables at home position
cVolL, cVolR, cVolT, cVolP = 0, 0, 0, 0
cableL, cableR, cableT = kineSolve.SIDE_LENGTH, kineSolve.SIDE_LENGTH, kineSolve.SIDE_LENGTH
prismP = 0
targetP = 0
[targetXideal, targetYideal, targetP, inclin, azimuth] = kineSolve.intersect(targetX, targetY, targetZ)
currentX = targetXideal
currentY = targetYideal
# print(targetXideal, targetYideal, targetP)
[targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetXideal, targetYideal)
targetP = 0
# print(targetL, targetR, targetT)
repJaco = cJaco
repJpinv = cJpinv

cableL, targetL = 18.75, 18.75
cableR, targetR = 18.75, 18.75

# Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, tStepL, LcRealL, angleL] = kineSolve.length2Vol(cableL, targetL)
[cVolR, tSpeedR, tStepR, LcRealR, angleR] = kineSolve.length2Vol(cableR, targetR)
[cVolT, tSpeedT, tStepT, LcRealT, angleT] = kineSolve.length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
fStepP = 0
tStepP = 0
LcRealP = tStepP/kineSolve.STEPS_PER_MM_PRI
[LStep, RStep, TStep, PStep] = kineSolve.freqScale(fStepL, fStepR, fStepT, fStepP)
LStep, RStep, TStep, PStep = 0, 0, 0, 0

# Set initial pressure and calibration variables
pressL, pressR, pressT, pressP = 100000, 100000, 0, 0
prevPressL, prevPressR, prevPressT, prevPressP = 0, 0, 0, 0
pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0
timeL, timeR, timeT, timeP = 0, 0, 0, 0
prevTimeL, prevTimeR, prevTimeT, prevTimeP = 0, 0, 0, 0
conLHS, conRHS, conTOP = 0, 0, 0
dLHS, dRHS, dTOP = 0, 0, 0
collisionAngle = 1j

# Current position
cStepL = 0*tStepL
cStepR = 0*tStepR
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
pneuPress = 2000

############################################################################
# Visual servoing variables
targetOpL, targetOpR, targetOpT, targetOpP = 0, 0, 0, 0


############################################################################
# Optitrack connection
useRigidBodies = False
optiTrackConnected = opTrack.optiConnect(useRigidBodies)

###############################################################
# Connect to Peripherals

# Create function to find available COM ports, listen to replies, and assign COM ports based on replies
print("Connecting to syringe pumps...")
pumpsConnected = False
[pumpCOMS, pumpSer, pumpNames, COMlist] = arduinoInterface.ardConnect()
print(pumpCOMS)

# if useVisionFeedback:
#     config_path = 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45short/'
#     pose_est = PoseEstimator(config_path)
#     pose_est.initialize()

# Set COM port for each pump by using its handshake key
if len(pumpCOMS) == 2:
    pumpsConnected = True
    print("...")
    lhsCOM = pumpCOMS[pumpNames[0]]
    rhsCOM = pumpCOMS[pumpNames[1]]
    # topCOM = pumpCOMS[pumpNames[2]]
    # priCOM = pumpCOMS[pumpNames[3]]
    # pneuCOM = pumpCOMS[pumpNames[4]]

    lhsSer = pumpSer[lhsCOM]
    rhsSer = pumpSer[rhsCOM]
    # topSer = pumpSer[topCOM]
    # priSer = pumpSer[priCOM]
    # pneuSer = pumpSer[pneuCOM]
# else:
    # use data from file
    # pass



try:
    if pumpsConnected:
        print("Pumps connected: ")
        ardIntLHS = arduinoInterface.ardInterfacer(pumpNames[0], lhsSer)
        reply = ardIntLHS.connect()
        print(reply)
        ardIntRHS = arduinoInterface.ardInterfacer(pumpNames[1], rhsSer)
        reply = ardIntRHS.connect()
        print(reply)
        # ardIntTOP = arduinoInterface.ardInterfacer(pumpNames[2], topSer)
        # reply = ardIntTOP.connect()
        # print(reply)
        # ardIntPRI = arduinoInterface.ardInterfacer(pumpNames[3], priSer)
        # reply = ardIntPRI.connect()
        # print(reply)

        # ardIntPNEU = arduinoInterface.ardInterfacer(pumpNames[4], pneuSer)
        # reply = ardIntPNEU.connect()
        # print(reply)

        #############################################################
        # Calibrate arduinos for zero volume - maintain negative pressure for 4 seconds
        calibL = False
        calibR = False
        calibT = True
        calibP = True

        # Has the mechanism been calibrated/want to run without calibration?:
        calibrated = False
        # Perform calibration:
        print("Zeroing hydraulic actuators...")
        while (not calibrated):
            if not calibL:
                [realStepL, pressL, timeL] = ardIntLHS.listenZero(calibL, pressL, timeL)
                print(realStepL, pressL)
                # print("LHS: ", calibL)
            # if not calibR:
            #     [realStepR, pressR, timeR] = ardIntRHS.listenZero(calibR, pressR, timeR)
            #     print(realStepR, pressR)
                # print("RHS: ", calibR)
            # [realStepT, pressT, timeT] = ardIntTOP.listenZero(calibT, pressT, timeT)
            # print(realStepT, pressT)
            # [realStepP, pressP, timeP] = ardIntPRI.listenZero(calibP, pressP, timeP)
            # print(realStepP, calibP)

            if (realStepL == "000000LHS"):
                calibL = True
            # if (realStepR == "000000RHS"):
            #     calibR = True
            # if (realStepT == "000000TOP"):
            #     calibT = True
            # if (realStepP == "0200PRI"):
            #     calibP = True

            if (calibL == 1):
                calibrated = True
                # Send 0s instead of StepNo and pressMed as signal that calibration done
                StepNoL, StepNoR, StepNoT, StepNoP, StepNoA = 0, 0, 0, 0, 0
                pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0

            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
            ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
            # ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
            # ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
            # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
            ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
            # Ensure same number of rows in position log file
            posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

    else:
        print("PUMPS NOT CONNECTED. RUNNING WITHOUT PUMPS.")

    print("Calibration done.")
    print("Beginning path following task.")
    

    ################################################################
    # Begin main loop
    while(flagStop == False):

        # Stay at first coord for number of cycles
        if delayCount < delayLim:
            delayCount += 1
            pathCounter = 0
        else:
            pathCounter += 1

        if not omni_connected:
        # Go sequentially through path coordinates
            # End test when last coords reached
            if pathCounter >= len(xPath)/2:
                pathCounter = 0
                # break
            else:
                XYZPathCoords = [xPath[pathCounter], yPath[pathCounter], zPath[pathCounter]]
            # print(XYZPathCoords)
            # XYZPathCoords = [15, 8.66025, 20]
        else:
            phntmOmni.getOmniCoords()
            [xMap, yMap, zMap] = phntmOmni.omniMap()
            XYZPathCoords = [xMap, yMap, zMap]

        # Ideal target points refer to non-discretised coords on parallel mechanism plane, otherwise, they are discretised.
        # XYZPathCoords are desired coords in 3D.
        [targetXideal, targetYideal, targetOpP, inclin, azimuth] = kineSolve.intersect(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2])
        # print("Open loop : ", targetXideal, targetYideal, targetOpP)

        # Return target cable lengths at target coords and jacobian at current coords
        [targetOpL, targetOpR, targetOpT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetXideal, targetYideal)

        # Get cable speeds using Jacobian at current point and calculation of input speed
        [lhsV, rhsV, topV, actualX, actualY] = kineSolve.cableSpeeds(currentX, currentY, targetXideal, targetYideal, cJaco, cJpinv)
        # print(lhsV, rhsV, topV, actualX, actualY)
        # Find actual target cable lengths based on scaled cable speeds that result in 'actual' coords
        [scaleTargL, scaleTargR, scaleTargT, repJaco, repJpinv] = kineSolve.cableLengths(currentX, currentY, actualX, actualY)

        # if useVisionFeedback == True:
        #     T_Rob_Inst_camera = pose_est.tip_pose()#4x4 homo matrix in MM
        #     if T_Rob_Inst_camera is not None:
        #         realX = T_Rob_Inst_camera[0,3]
        #         realY = T_Rob_Inst_camera[1,3]
        #         realZ = T_Rob_Inst_camera[2,3]
        #         print("Open loop : ", targetXideal, targetYideal, targetOpP)
        #         # print("Position", -realZ + 15, realY + 8.66, realX)
        #         [errCableL, errCableR, errCableT, errPrism] = kineSolve.cableError(actualX, actualY, scaleTargL, scaleTargR, scaleTargT, targetOpP, realX, realY, realZ)
        #         print("OL cables : ", targetOpL, targetOpR, targetOpT, targetOpP)
        #         print("Error LRTP: ", errCableL, errCableR, errCableT, errPrism)

        #     if visionFeedFlag:
        #         print("Closed Loop active \n")
        #         targetL = scaleTargL - errCableL
        #         targetR = scaleTargR - errCableR
        #         targetT = scaleTargT - errCableT
        #         targetP = targetOpP - errPrism

        # else:
        targetL = scaleTargL
        targetR = scaleTargR 
        targetT = scaleTargT
        targetP = targetOpP

        tStepP = int(targetP*kineSolve.STEPS_PER_MM_PRI)
        tStepP += targDir*antiHystSteps
        LcRealP = tStepP/kineSolve.STEPS_PER_MM_PRI
        # For anti-hysteresis in prismatic joint, check target direction and current direction of motion:
        if tStepP > cStepP:
            targDir = 1
        elif tStepP < cStepP:
            targDir = -1
        # If stopped, preserve previous direction as target direction: 
        elif tStepP == cStepP:
            targDir = cDir
        # print(targetL, targetR, targetT, LcRealP)

        cableL, targetL = 18.75, 18.75
        cableR, targetR = 18.75, 18.75
        # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
        [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
        # print(LcRealL, LcRealR, LcRealT, LcRealP)

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



        # print(StepNoL, StepNoR, StepNoT, StepNoP)

        # Log deisred position at 
        # posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

        if optiTrackConnected:
            [pos_x, vel_x] = energyS.trackToState(opTrack.markerData[-1])

        if energyFlag:
            print("Distance: ", round(pos_x*1000,3), " Velocity: ", round(vel_x*1000,3), "Error: ", round((pos_x - x_d)*1000,3))
            [controlInputs, vol_est_1, vol_est_2] = energyS.energyShape(pos_x, vel_x, pressL, pressR, x_d, dt, 5, 0, 1)
            print("Predicted stepper pos: ", vol_est_1*kineSolve.M3_to_MM3*kineSolve.STEPS_PER_MMCUBED, vol_est_2*kineSolve.M3_to_MM3*kineSolve.STEPS_PER_MMCUBED)
            # target_1  = vol_est_1*kineSolve.M3_to_MM3*kineSolve.STEPS_PER_MMCUBED
            # target_2  = vol_est_2*kineSolve.M3_to_MM3*kineSolve.STEPS_PER_MMCUBED
            print("U1: ", controlInputs[0], " U2: ", controlInputs[1], "F: ", controlInputs[2])
            # print("U1: ", round(controlInputs[0], 3), " U2: ", round(controlInputs[1], 3), "F: ", round(controlInputs[2], 3))
            print("Pressures: ", pressL, pressR)
            [StepNoL , StepNoR] = energyS.traject(int(realStepL), int(realStepR), dt)
            # [StepNoL , StepNoR] = energyS.traject(StepNoL, StepNoR, dt)
            # print("StepNos: ", StepNoL , StepNoR)

            StepNos = [StepNoL, StepNoR]
            for i in range(len(StepNos)):
                if StepNos[i] < 0:
                    StepNos[i] = 100
                elif StepNos[i] > kineSolve.MAX_STEPS:
                    StepNos[i] = int(kineSolve.MAX_STEPS)
            
            StepNoL = StepNos[0]
            StepNoR = StepNos[1]

            print("StepNos: ", StepNoL , StepNoR)

        else:
            StepNoL = int(tStepL*0.95)
            StepNoR = int(tStepR*0.95)

        posLogging.posLog(pos_x, vel_x, controlInputs[0], controlInputs[1], controlInputs[2])

        if pumpsConnected:

            # Calculate median pressure over 10 samples:
            pressLMed = ardIntLHS.newPressMed(pressL)
            pressRMed = ardIntRHS.newPressMed(pressR)
            # pressTMed = ardIntTOP.newPressMed(pressT)
            # pressAMed = ardIntPNEU.newPressMed(pressA)
            [conLHS, dLHS] = ardIntLHS.derivPress(timeL, prevTimeL)
            [conRHS, dRHS] = ardIntRHS.derivPress(timeR, prevTimeR)
            # [conTOP, dTOP] = ardIntTOP.derivPress(timeT, prevTimeT)
            collisionAngle = kineSolve.collisionAngle(dLHS, dRHS, dTOP, conLHS, conRHS, conTOP)

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
                # ardIntTOP.sendStep(initStepNoT)
                # ardIntPRI.sendStep(StepNoP)
            else:
                if useVisionFeedback:
                    visionFeedFlag = 1
                if useEnergyShaping:
                    energyFlag = True
                # Send step number to arduinos:
                ardIntLHS.sendStep(StepNoL)
                ardIntRHS.sendStep(StepNoR)
                # ardIntTOP.sendStep(StepNoT)
                # ardIntPRI.sendStep(StepNoP)

            # Log values from arduinos
            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
            ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
            # ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
            # ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
            # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
            ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)

            # Get current pump position, pressure and times from arduinos
            [realStepL, pressL, timeL] = ardIntLHS.listenReply()
            [realStepR, pressR, timeR] = ardIntRHS.listenReply()
            # if (realStepL == "S_Empty"):
            #     realStepL = StepNoL
            # elif (realStepR == "S_Empty"):
            #     realStepR = StepNoR

            # if pressL == "P_Empty":
            #     pressL = pressLMed
            # elif pressR == "P_Empty":
            #     pressR = pressRMed
            # [realStepT, pressT, timeT] = ardIntTOP.listenReply()
            # [realStepP, pressP, timeP] = ardIntPRI.listenReply()
            # [realStepA, pressA, timeA] = ardIntPNEU.listenReply()

            # Check for high pressure
            if (max(int(pressL), int(pressR)) > PRESS_MAX_PA):
                print("Overpressure: ", max(int(pressL), int(pressR)), " Pa")
                flagStop = True
            elif (min(int(pressL), int(pressR)) < PRESS_MIN_PA):
                print("Under pressure: ", min(int(pressL), int(pressR)), " Pa")
                flagStop = True

            
            dt = (timeL - prevTimeL)/1000 # kineSolve.TIMESTEP
        else:
            dt = kineSolve.TIMESTEP

        # Update current position, cable lengths, and volumes as previous targets
        currentX = actualX
        currentY = actualY
        cableL = targetL
        cableR = targetR
        cableT = targetT
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
        prevPathCounter = pathCounter

        # Close GUI if Esc hit
        # flagStop = False # Will close immediately 



except TypeError as exTE:
    tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
    tb_textTE = ''.join(tb_linesTE)
    print(tb_textTE)

    # except Exception as ex:
    #     tb_lines = traceback.format_exception(ex.__class__, ex, ex.__traceback__)
    #     tb_text = ''.join(tb_lines)
    #     print(tb_text)
    

finally:
    ###########################################################################
    # Stop program
    # Disable pumps and set them to idle state
    # print(kineSolve.MAX_VOL/kineSolve.M3_to_MM3, kineSolve.MAX_STEPS, kineSolve.STEPS_PER_MMCUBED)

    # try:


    if pumpsConnected:
        # Save values gathered from arduinos
        ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
        ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
        # ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
        # ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
        # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
        ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
        ardLogging.ardSave()
        # Ensure same number of rows in position log file
        posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

    #Save position data
    posLogging.posSave()

    if 'ardIntLHS' in locals():
        if ardIntLHS.ser.is_open:
            ardIntLHS.sendStep(CLOSEMESSAGE)
            time.sleep(0.2)
            [realStepL, pressL, timeL] = ardIntLHS.listenReply()
            print(realStepL, pressL, timeL)
            ardIntLHS.ser.close()

    if 'ardIntRHS' in locals():
        if ardIntRHS.ser.is_open:
            ardIntRHS.sendStep(CLOSEMESSAGE)
            time.sleep(0.2)
            [realStepR, pressR, timeR] = ardIntRHS.listenReply()
            print(realStepR, pressR, timeR)
            ardIntRHS.ser.close()
            
    # print(pumpSer)
    for i in pumpSer:
        print(pumpSer[i])
        pumpSer[i].__exit__()

        # if ardIntTOP.ser.is_open:
        #     ardIntTOP.sendStep(CLOSEMESSAGE)

        # if ardIntPRI.ser.is_open:
        #     ardIntPRI.sendStep(CLOSEMESSAGE)

        # if ardIntPNEU.ser.is_open:
        #     ardIntPNEU.sendStep(CLOSEMESSAGE)

    # #Save optitrack data
    # if optiTrackConnected:
    if useRigidBodies:
        opTrack.optiSave(opTrack.rigidData)
    else:
        opTrack.optiSave(opTrack.markerData)
    opTrack.optiClose()
    while opTrack.trackSock.stop_threads == False:
        opTrack.trackSock.stop_threads = True


        # time.sleep(0.2)
        # [realStepT, pressT, timeT] = ardIntTOP.listenReply()
        # print(realStepT, pressT, timeT)
        # time.sleep(0.2)
        # [realStepP, pressP, timeP] = ardIntPRI.listenReply()
        # print(realStepP, pressP, timeP)
        # time.sleep(0.2)
        # [realStepA, pressA, timeA] = ardIntPNEU.listenReply()
        # print(realStepA, pressA, timeA)


        # Close serial connections
        # ardIntTOP.ser.close()
        # ardIntPRI.ser.close()
        # ardIntPNEU.ser.close()

    # except TypeError as exTE:
    #     tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
    #     tb_textTE = ''.join(tb_linesTE)
        # print(tb_textTE)

# if __name__ == '__main__':
#     pass