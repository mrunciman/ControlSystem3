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
from modules import fibrebotInterface
from modules import massSpecInterface
from modules import kinematics
# from modules import mouseGUI
from modules import pumpLog
from modules import positionInput
from modules import optiStream
from modules import omniStream
from modules import clusterData
from modules import threadArdComms

from visual_navigation.cam_pose import PoseEstimator

np.set_printoptions(suppress=True, precision = 2)
############################################################
# Instantiate classes:
sideLength = 40 # mm, from workspace2 model

kineSolve = kinematics.kineSolver(sideLength)
# mouseTrack = mouseGUI.mouseTracker(sideLength)
ardLogging = pumpLog.ardLogger()
posLogging = positionInput.posLogger()
opTrack = optiStream.optiTracker()
phntmOmni = omniStream.omniStreamer()
dataClust = clusterData.dataClustering()

pumpController = threadArdComms.ardThreader()
pumpDataUpdated = False

CALIBRATION_MODE = 0
HOLD_MODE = 1
ACTIVE_MODE = 2
INFLATION_MODE = 0
DEFLATION_MODE = 1
SET_PRESS_MODE = 3


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
print("Haptic device connected? ", omni_connected)

# Try to connect to phantom omni. If not connected, use pre-determined coords.
if not omni_connected:
    with open('control/paths/gridPath 2023-03-03 16-29-08 centre 15-8.66025 30x15.0grid 0.048x1.5spacing.csv', newline = '') as csvPath:
        coordReader = csv.reader(csvPath)
        for row in coordReader:
            xPath.append(float(row[0]))
            yPath.append(float(row[1]))
            zPath.append(float(row[2]))
        xMap, yMap, zMap = xPath[0], yPath[0], zPath[0]
else:
    # omniX, omniY, omniZ = 0.0, 0.0, 0.0
    omniDataReceived = phntmOmni.getOmniCoords()
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
PRESS_MAX_KPA = 190000
flagStop = False

# Desired pressure in pneumatic structure
regulatorPressure = 100
regulatorSensor = 0

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
delayEveryStep = True
delayFactor = 1
firstMoveDelay = 0
firstMoveDivider = 100
initialXFlag = False
initPressLogCount = 0
initPressLogNum = 10
useVisionFeedback = False
visionFeedFlag = False

# Fibre related variables
fibreDone = False
pauseVisFeedback = False

behaviourState = 1

MSCounter = 0
miniPathCounter = 0
numClusters = 0

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
pneuPress = 2000

############################################################################
# Visual servoing variables
targetOpL, targetOpR, targetOpT, targetOpP = 0, 0, 0, 0
errCableL, errCableR, errCableT, errPrism = 0, 0, 0, 0

############################################################################
# Optitrack connection
useRigidBodies = True
optiTrackConnected = False
# optiTrackConnected = opTrack.optiConnect()
# optiTrackConnected = True

###############################################################
# Connect to Peripherals

# Create function to find available COM ports, listen to replies, and assign COM ports based on replies

# startThreader opens the serial connection and starts the communication thread
pumpController.startThreader()
pumpsConnected = pumpController.connected
print("Connected to controller? ", pumpsConnected)

if pumpsConnected:
    pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, DEFLATION_MODE)
# [pumpCOMS, pumpSer, pumpNames, COMlist] = arduinoInterface.ardConnect()
# print(pumpCOMS)

fibrebotLink = fibrebotInterface.fibreBot()
fibreConnected = fibrebotLink.connect()
# if fibreConnected:
print("Fibrebot connected? ", fibreConnected)
    # print(fibrebotLink.fibreSerial)

massSpecLink = massSpecInterface.massSpec()
msCOM = 'COM5'
msConnected = massSpecLink.connect(msCOM)
# if msConnected:
print("Mass spec serial connected? ", msConnected)

config_path = 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45short/'
pose_est = PoseEstimator(config_path)
if useVisionFeedback:
    # config_path = 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/visual_navigation/data_45short/'
    # pose_est = PoseEstimator(config_path)
    pose_est.initialize()
else:
    pose_est.camConnected = False
print("Use camera?", pose_est.camConnected)



CLOSEMESSAGE = "Closed"

try:
    if pumpsConnected:

        #  Inflate structure and give some time to stabilise:
        print("Inflating structure...")
        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, INFLATION_MODE)
        time.sleep(3)

        # Has the mechanism been calibrated/want to run without calibration?:
        calibrated = True
        # Perform calibration:
        print("Zeroing hydraulic actuators...")
        if (not calibrated):
            pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, INFLATION_MODE)
        while (not calibrated):

            time.sleep(0.007)
            [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
            [timeL, timeR, timeT, timeP] = [timeL]*4

            if (pumpController.calibrationFlag == 'Y'):
                calibrated = True
                # Send 0s instead of StepNo and pressMed as signal that calibration done
                StepNoL, StepNoR, StepNoT, StepNoP, StepNoA = 0, 0, 0, 0, 0
                pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0

            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
            ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
            ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
            ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
            # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
            ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
            # Ensure same number of rows in position log file
            posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)
        print("Calibration done.")

    else:
        print("PUMP CONTROLLER NOT CONNECTED. RUNNING WITHOUT PUMPS.")

    print("Beginning path following task.")
    if fibreConnected: fibrebotLink.sendState("Run")
    

    ################################################################
    # Begin main loop
    while(flagStop == False):

        if not omni_connected:
        # CHOOSE WHICH BEHAVIOUR TO EXECUTE
            # Gross raster until end of path
            if pathCounter >= len(xPath)/18:
                break               
            else:
                XYZPathCoords = [xPath[pathCounter], yPath[pathCounter], zPath[pathCounter]]
                # print(XYZPathCoords)
        else:
            omniDataReceived = phntmOmni.getOmniCoords()
            # if not omniDataReceived: break
            [xMap, yMap, zMap] = phntmOmni.omniMap()
            XYZPathCoords = [xMap, yMap, zMap]
            # print(XYZPathCoords)
         
        # Ideal target points refer to non-discretised coords on parallel mechanism plane, otherwise, they are discretised.
        # XYZPathCoords are desired coords in 3D.
        [targetXideal, targetYideal, targetOpP, inclin, azimuth] = kineSolve.intersect(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2])
        # print("Open loop : ", targetXideal, targetYideal, targetOpP)

        # Return target cable lengths at target coords and jacobian at current coords
        [targetOpL, targetOpR, targetOpT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetXideal, targetYideal)

        # Get cable speeds using Jacobian at current point and calculation of input speed
        [lhsV, rhsV, topV, actualX, actualY, perpAngle] = kineSolve.cableSpeeds(currentX, currentY, targetXideal, targetYideal, cJaco, cJpinv)
        fibrebotLink.lineAngle = perpAngle

        # Find actual target cable lengths based on scaled cable speeds that result in 'actual' coords
        [scaleTargL, scaleTargR, scaleTargT, repJaco, repJpinv] = kineSolve.cableLengths(currentX, currentY, actualX, actualY)

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
        # print(lhsV, rhsV, topV, actualX, actualY)
        # print(cVolL, cVolR, cVolT)    

        # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
        # Use current volume, current cable length and target cable length
        [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
        # print(tVolL, tVolR, tVolT)

        desiredThetaL = kineSolve.volToAngle(tVolL)
        desiredThetaR = kineSolve.volToAngle(tVolR)
        desiredThetaT = kineSolve.volToAngle(tVolT)
        desiredThetaP = 360.0*targetOpP/kineSolve.LEAD
        print(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP)

        [tVolL, tVolR, tVolT] = kineSolve.volRateScale(tVolL, tVolR, tVolT, cVolL, cVolR, cVolT)

        # # CALCULATE FREQS FROM VALID STEP NUMBER
        # # tStepL is target pump position, cStepL is current, speed controlled position.
        # fStepL = (tStepL - cStepL)*SAMP_FREQ
        # fStepR = (tStepR - cStepR)*SAMP_FREQ
        # fStepT = (tStepT - cStepT)*SAMP_FREQ
        # fStepP = (tStepP - cStepP)*SAMP_FREQ
        # [LStep, RStep, TStep, PStep] = kineSolve.freqScale(fStepL, fStepR, fStepT, fStepP)
        # # RStep = dStepR scaled for speed (w rounding differences)
        # StepNoL += LStep
        # StepNoR += RStep 
        # StepNoT += TStep
        # StepNoP += PStep
        # print(StepNoL, StepNoR, StepNoT, StepNoP)

        # Log deisred position at 
        posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

        if pumpsConnected:
        # Reduce speed when making first move after calibration.
            if firstMoveDelay < firstMoveDivider:
                firstMoveDelay += 1
                # RStep = dStepR scaled for speed (w rounding differences)
                initStepNoL = int(desiredThetaL*(firstMoveDelay/firstMoveDivider))
                initStepNoR = int(desiredThetaR*(firstMoveDelay/firstMoveDivider))
                initStepNoT = int(desiredThetaT*(firstMoveDelay/firstMoveDivider))
                initStepNoP = int(desiredThetaP*(firstMoveDelay/firstMoveDivider))
                # TODO change sendStep args to desiredAngleL etc
                # Send scaled step number to arduinos:
                pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, initStepNoP, regulatorPressure, ACTIVE_MODE, INFLATION_MODE)
            else:
                # Send step number to arduinos:
                pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, ACTIVE_MODE, INFLATION_MODE)

            # Log values from arduinos
            if pumpDataUpdated:
                ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
                ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
                ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
                ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
                # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)

            # Get current pump position, pressure and times from arduinos
            [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
            [timeL, timeR, timeT, timeP] = [timeL]*4



            # Check for high pressure
            if (max(pressLMed, pressRMed, pressTMed, pressPMed) > PRESS_MAX_KPA):
                print("Overpressure: ", max(pressL, pressR, pressT, pressL), " kPa")
                flagStop = True

        # Check if new data has been received from pump controller 
        if (timeL - prevTimeL > 0):
            pumpDataUpdated = True
            kineSolve.TIMESTEP = (timeL - prevTimeL)/1000
            if (kineSolve.TIMESTEP < 0.01): kineSolve.TIMESTEP = 0.01
            # print((timeL - prevTimeL)/1000)
        else:
            pumpDataUpdated = False
            kineSolve.TIMESTEP = 6/125

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
        if pumpDataUpdated: pathCounter += 1

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
    try:


        if pumpsConnected:
            # Save values gathered from arduinos
            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL)
            ardLogging.ardLog(realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed, timeR)
            ardLogging.ardLog(realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT)
            ardLogging.ardLog(realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP)
            # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
            ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
            ardLogging.ardSave()
            # Ensure same number of rows in position log file
            posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)


        #Save pose estimation data
        massSpecLink.savePoseMassSpec()

        #Save position data
        posLogging.posSave()

        # #Save optitrack data
        if optiTrackConnected:
            if useRigidBodies:
                opTrack.optiSave(opTrack.rigidData)
            else:
                opTrack.optiSave(opTrack.markerData)


        if pumpsConnected:
            pumpController.sendStep(cStepL, cStepR, cStepT, cStepP, regulatorPressure, HOLD_MODE, DEFLATION_MODE)
            pumpController.stopThreader()
            time.sleep(0.2)
            [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
            [timeL, timeR, timeT, timeP] = [timeL]*4
            pumpController.closeSerial()
            print("Connection to pump controller closed.")
            print(realStepL, pressL, timeL)
            print(realStepR, pressR, timeR)
            print(realStepT, pressT, timeT)
            print(realStepP, pressP, timeP)
            pumpController.t.stop()
            pumpController.ser.close

        
        if optiTrackConnected:
            opTrack.optiClose()

        if fibreConnected:
            # Send stop message to fibrebot
            fibrebotLink.sendState("Stop")
            fibrebotLink.fibreSerial.close()

        if omni_connected:
            phntmOmni.omniClose()

    except TypeError as exTE:
        tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
        tb_textTE = ''.join(tb_linesTE)
        # print(tb_textTE)

# if __name__ == '__main__':
#     pass