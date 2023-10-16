

import csv
import traceback
import time
import numpy as np 
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
from functools import partial
import threading
import sv_ttk

np.set_printoptions(suppress=True, precision = 2)

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



######################################################################
def moveRobot(dictButtons, dictLabel, dictPress, classSettings):
    print("'Move robot' button pressed")

    minPress = -15
    deactivateButtons(dictButtons)
    
    [useVisionFeedback, visionFeedFlag, startWithCalibration, useOmni, socketOmni, useOptitrack, useFibrebot, useMassSpec, usePathFile, flagStop]\
        = list(vars(classSettings).values())
    
    print("Settings: ", vars(classSettings))


    ############################################################
    # Instantiate classes:
    # sideLength = 18.78 # mm, from workspace2 model
    sideLength = 38 # mm, from workspace2 model

    kineSolve = kinematics.kineSolver(sideLength)
    # mouseTrack = mouseGUI.mouseTracker(sideLength)
    ardLogging = pumpLog.ardLogger()
    posLogging = positionInput.posLogger()
    opTrack = optiStream.optiTracker()
    phntmOmni = omniStream.omniStreamer()
    dataClust = clusterData.dataClustering()
    pressDetector = arduinoInterface.ardInterfacer
    pumpController = threadArdComms.ardThreader()
    medPressL = kinematics.forceDetector()
    medPressR = kinematics.forceDetector()
    medPressT = kinematics.forceDetector()
    medPressP = kinematics.forceDetector()

    SAMP_FREQ = 1/kineSolve.TIMESTEP
    CALIBRATION_MODE = 0
    HOLD_MODE = 1
    ACTIVE_MODE = 2
    INFLATION_MODE = 0
    DEFLATION_MODE = 1
    SET_PRESS_MODE = 3

    # Other constants
    # PRESS_MAX_KPA = 900
    VAC_PRESS = -15

    CLOSEMESSAGE = "Closed"


    calibrated = not startWithCalibration
    pumpDataUpdated = False
    delayFactor = 1
    firstMoveDelay = 0
    firstMoveDivider = 400
    delayCount = 0
    delayLim = 200

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




    ############################################################################
    # Initialise variables 

    flagStop = False

    # Desired pressure in pneumatic structure
    regulatorPressure = 0
    inflationPressure = 100
    regulatorSensor = 0
    

    # Use different methods for different paths
    xPath = []
    yPath = []
    zPath = []

    if useOmni:
        if socketOmni is None:
            omni_connected = False
        else: 
            omni_connected = True
            phntmOmni.sock = classSettings.socketOmni
        omni_connected = phntmOmni.connectOmni(omni_connected)
        print("Haptic device connected? ", omni_connected)
    else: 
        omni_connected = False
    
    if omni_connected:
        dictLabel["omniLabel"].config(fg = "green") 
    else:
        dictLabel["omniLabel"].config(fg = "red")
    
    omniButtons = phntmOmni.omniButton # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both

    # Try to connect to phantom omni. If not connected, use pre-determined coords.
    # if usePathFile:
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

    # Target must be cast as immutable type (float, in this case) so that 
    # the current position doesn't update at same time as target
    currentX = XYZPathCoords[0]
    currentY = XYZPathCoords[1]
    currentZ = XYZPathCoords[2]
    targetX = XYZPathCoords[0]
    targetY = XYZPathCoords[1]
    targetZ = XYZPathCoords[2]


    # Fibre related variables
    fibreDone = False
    pauseVisFeedback = False

    behaviourState = 1

    MSCounter = 0
    miniPathCounter = 0
    numClusters = 0

    # Initialise cable length variables 
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

    #Cable length to centre of triangle is sideLength/(3**0.5) # tan(30) is 1/sqrt(3)
    [targetL, targetR, targetT, targetP] = sideLength/(3**0.5), sideLength/(3**0.5), sideLength/(3**0.5), 10  # Cable length to centre of scaffold

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

    desiredThetaL = kineSolve.volToAngle(tVolL)
    desiredThetaR = kineSolve.volToAngle(tVolR)
    desiredThetaT = kineSolve.volToAngle(tVolT)
    desiredThetaP = 360.0*targetP/kineSolve.LEAD

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
    if useOptitrack:
        optiTrackConnected = opTrack.optiConnect()
    # optiTrackConnected = True

    ###############################################################
    # Connect to Peripherals

    # Create function to find available COM ports, listen to replies, and assign COM ports based on replies

    # startThreader opens the serial connection and starts the communication thread
    pumpController.startThreader()
    pumpsConnected = pumpController.connected
    print("Connected to controller? ", pumpsConnected)
    
    dictLabel["pumpLabel"].config(fg = "green") if pumpsConnected else dictLabel["pumpLabel"].config(fg = "red")

    if pumpsConnected:
        omniButtons = phntmOmni.omniButton
        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, DEFLATION_MODE, omniButtons)
    # [pumpCOMS, pumpSer, pumpNames, COMlist] = arduinoInterface.ardConnect()
    # print(pumpCOMS)

    fibrebotLink = fibrebotInterface.fibreBot()
    fibreConnected = False
    if useFibrebot:
        fibreConnected = fibrebotLink.connect()
        # if fibreConnected:
        print("Fibrebot connected? ", fibreConnected)
            # print(fibrebotLink.fibreSerial)

    massSpecLink = massSpecInterface.massSpec()
    msCOM = 'COM5'
    msConnected = False
    if useMassSpec:
        msConnected = massSpecLink.connect(msCOM)
        # if msConnected:
        print("Mass spec serial connected? ", msConnected)

    try:
        if startWithCalibration:

            if pumpsConnected:

                #  Inflate structure and give some time to stabilise:
                print("Inflating structure...")
                time.sleep(1)
                omniButtons = phntmOmni.omniButton
                pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE,omniButtons)
                # count = 0
                # countLimit = 1000
                # rampTime = 3 # seconds
                # while (count != countLimit):
                #     regulatorPressure = inflationPressure*(count/countLimit)
                #     time.sleep(rampTime/countLimit)
                #     omniButtons = phntmOmni.omniButton
                #     pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, omniButtons)
                #     [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
                #     pressList = [pressL, pressR, pressT, regulatorSensor]
                #     # Change pressurebars
                #     updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)
                #     count = count + 1

                #     # Stop operation if Stop button hit
                #     flagStop = classSettings.stopFlag
                #     if flagStop == True:
                #         activateButtons(dictButtons, flagStop)
                #         raise 
                # # Wait an additional 3 s to stabilise
                # time.sleep(1)


                # Has the mechanism been calibrated/want to run without calibration?:
                calibrated = not startWithCalibration



                if (not calibrated):
                    # Perform calibration:
                    print("Zeroing hydraulic actuators...")
                    dictLabel["calibrationLabel"].config(fg = "red")
                    pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, SET_PRESS_MODE, omniButtons)
                    
                while (not calibrated):

                    # Stop operation if Stop button hit
                    flagStop = classSettings.stopFlag
                    if flagStop == True:
                        activateButtons(dictButtons, flagStop)
                        raise

                    if (max(pressL, pressR, pressT) > PRESS_MAX_KPA): # TODO Add filtered pressure values back again to use here
                        print("Overpressure: ", max(pressL, pressR, pressT), " kPa")
                        raise

                    [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
                    [timeL, timeR, timeT, timeP] = [timeL]*4
                    # print(pumpController.calibrationFlag)
                    # print(pressL, pressR, pressT, regulatorSensor, "\n")

                    pressList = [pressL, pressR, pressT, regulatorSensor]
                    # Change pressurebars
                    updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)

                    if omni_connected:
                        omniDataReceived = phntmOmni.getOmniCoords()
                        omniButtons = phntmOmni.omniButton

                    if (pumpController.calibrationFlag == 'Y'):
                        dictLabel["calibrationLabel"].config(fg = "green")
                        calibrated = True
                        # Send 0s instead of desiredTheta and pressMed as signal that calibration done
                        desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, StepNoA = 0, 0, 0, 0, 0
                        pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0
                    else:
                        time.sleep(0.007)
                        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, SET_PRESS_MODE, omniButtons)

                    ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, timeL)
                    ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, timeR)
                    ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, timeT)
                    ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, timeP)
                    # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                    ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
                    # Ensure same number of rows in position log file
                    posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)
                print("Calibration done.")

            else:
                print("PUMP CONTROLLER NOT CONNECTED. RUNNING WITHOUT PUMPS.")

        # print("Beginning path following task.")
        if fibreConnected: fibrebotLink.sendState("Run")


        if startWithCalibration and not calibrated and pumpsConnected:
            dictButtons['moveButton'].config(bg = '#A877BA')
            raise

        if not messagebox.askokcancel("Proceed?", "Start the robot?"):
            raise

        ################################################################
        # Begin main loop
        while(flagStop == False):

            if not omni_connected:
            # CHOOSE WHICH BEHAVIOUR TO EXECUTE
                # Gross raster until end of path
                if pathCounter >= len(xPath)/9:
                    break               
                else:
                    XYZPathCoords = [xPath[pathCounter], yPath[pathCounter], zPath[pathCounter]]
                    # print(XYZPathCoords)
            else:
                omniDataReceived = phntmOmni.getOmniCoords()
                omniButtons = phntmOmni.omniButton
                if (omniDataReceived == 2): break
                [xMap, yMap, zMap] = phntmOmni.omniMap()
                XYZPathCoords = [xMap, yMap, zMap]
                # print(XYZPathCoords)
            
            # Ideal target points refer to non-discretised coords on parallel mechanism plane, otherwise, they are discretised.
            # XYZPathCoords are desired coords in 3D.
            [targetXideal, targetYideal, targetOpP, inclin, azimuth] = kineSolve.intersect(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2])
            # [targetXideal, targetYideal, targetOpP] = 0, 0, 10 # HOMING ONLY  [0, -13.47, 50.4]

            # if not omni_connected: 
            #     [targetXideal, targetYideal, targetOpP] = 0, 0, 10
                # print("Haptic not connected")
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
            if not omni_connected: 
                [targetL, targetR, targetT, targetP] = 20.18931022920578, 20.18931022920578, 20.189310229205777, 10 # Homing
            # [targetL, targetR, targetT, targetP] = 20.18931022920578, 20.18931022920578, 20.189310229205777, 10 # Homing


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
            # print(targetL, targetR, targetT, targetP)
            # print(lhsV, rhsV, topV, actualX, actualY)
            # print(cVolL, cVolR, cVolT)

            # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
            # Use current volume, current cable length and target cable length
            [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
            [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
            [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
            # print("\n",tStepL, tStepR, tStepT, "\n")
            # print("\nVolumes in ml: ",tVolL/1000, tVolR/1000, tVolT/1000, targetOpP, "\n")

            [tVolL_Scaled, tVolR_Scaled, tVolT_Scaled] = kineSolve.volRateScale(tVolL, tVolR, tVolT, cVolL, cVolR, cVolT)

            desiredThetaL = kineSolve.volToAngle(tVolL_Scaled)
            desiredThetaR = kineSolve.volToAngle(tVolR_Scaled)
            desiredThetaT = kineSolve.volToAngle(tVolT_Scaled)
            desiredThetaP = 360.0*targetOpP/kineSolve.LEAD
            # print(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, "\n")

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

            # Log desired positions
            if pumpDataUpdated:
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

            if pumpsConnected:
                if startWithCalibration:
                    # Reduce speed when making first move after calibration.
                    if firstMoveDelay < firstMoveDivider:
                        firstMoveDelay += 1
                        # RStep = dStepR scaled for speed (w rounding differences)
                        initStepNoL = int(desiredThetaL*(firstMoveDelay/firstMoveDivider))
                        initStepNoR = int(desiredThetaR*(firstMoveDelay/firstMoveDivider))
                        initStepNoT = int(desiredThetaT*(firstMoveDelay/firstMoveDivider))
                        initStepNoP = int(desiredThetaP*(firstMoveDelay/firstMoveDivider))
                        # Send scaled step number to arduinos:
                        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, initStepNoP, regulatorPressure, ACTIVE_MODE, SET_PRESS_MODE, omniButtons)
                    else:
                        # Send step number to arduinos:
                        pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, ACTIVE_MODE, SET_PRESS_MODE, omniButtons)

                # Log values from arduinos
                if pumpDataUpdated:
                    ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, timeL)
                    ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, timeR)
                    ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, timeT)
                    ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, timeP)
                    # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                    ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)

                # Get current pump position, pressure and times from arduinos
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
                [timeL, timeR, timeT, timeP] = [timeL]*4

                pressLMed = medPressL.newPressMed(pressL)
                pressRMed = medPressR.newPressMed(pressR)
                pressTMed = medPressT.newPressMed(pressT)
                pressP = regulatorSensor
                pressPMed = medPressP.newPressMed(regulatorSensor)

                # pressList = [pressL, pressR, pressT, regulatorSensor]
                pressList = [pressLMed, pressRMed, pressTMed, pressPMed]
 
                # Change pressurebars
                updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)

                # Check for high pressure
                if (max(pressL, pressR, pressT) > PRESS_MAX_KPA): # TODO Add filtered pressure values back again to use here
                    print("Overpressure: ", max(pressL, pressR, pressT), " kPa")
                    break

            # Check if new data has been received from pump controller 
            if (timeL - prevTimeL > 0):
                pumpDataUpdated = True
                kineSolve.TIMESTEP = (timeL - prevTimeL)/1000
                kineSolve.TIMESTEP = 0.01
                if (kineSolve.TIMESTEP < 0.01): kineSolve.TIMESTEP = 0.01
                # print((timeL - prevTimeL)/1000)
            else:
                pumpDataUpdated = False
                kineSolve.TIMESTEP = 0.01

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
            # pathCounter += 1
            # if pumpDataUpdated: pathCounter += 1

            # Stop operation if Stop button hit
            flagStop = classSettings.stopFlag


    except TypeError as exTE:
        tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
        tb_textTE = ''.join(tb_linesTE)
        print(tb_textTE)

    except Exception as ex:
        tb_lines = traceback.format_exception(ex.__class__, ex, ex.__traceback__)
        tb_text = ''.join(tb_lines)
        print(tb_text)
        

    finally:
        # Control loop is over
        # Reactivate selection buttons 
        activateButtons(dictButtons, flagStop)

        print("Ending control loop...")

        ###########################################################################
        # Stop program
        # Disable pumps and set them to idle state
        try:

            if pumpsConnected:
                # Save values gathered from arduinos
                ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, timeL)
                ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, timeR)
                ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, timeT)
                ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, timeP)
                # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
                ardLogging.ardSave()
                # Ensure same number of rows in position log file
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

                # if calibrated:
                omniButtons = 0
                pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, HOLD_MODE, DEFLATION_MODE, omniButtons)
                pumpController.stopThreader()
                time.sleep(0.2)
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
                [timeL, timeR, timeT, timeP] = [timeL]*4
                print("Connection to pump controller closed.")
                print(realStepL, pressL, timeL)
                print(realStepR, pressR, timeR)
                print(realStepT, pressT, timeT)
                print(realStepP, pressP, timeP)
                # pumpController.t.stop()
                pumpController.closeSerial()



            #Save position data
            posLogging.posSave()


            #Save pose estimation data
            massSpecLink.savePoseMassSpec()


            # #Save optitrack data
            if optiTrackConnected:
                if useRigidBodies:
                    opTrack.optiSave(opTrack.rigidData)
                else:
                    opTrack.optiSave(opTrack.markerData)
                opTrack.optiClose()


            if fibreConnected:
                # Send stop message to fibrebot
                fibrebotLink.sendState("Stop")
                fibrebotLink.fibreSerial.close()


            if omni_connected:
                # phntmOmni.omniClose()
                classSettings.socketOmni = phntmOmni.sock


        except TypeError as exTE:
            tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
            tb_textTE = ''.join(tb_linesTE)
            print(tb_textTE)
        
        
        print("Move Robot complete.")





###################################################################################################
###################################################################################################


class controlSettings:
    def __init__(self):
        self.useVisionFeedback = False
        self.visionFeedFlag = False
        self.startWithCalibration = True
        self.useOmni = True
        self.socketOmni = None
        self.useOptitrack = False
        self.useFibrebot = False
        self.useMassSpec = False
        self.usePathFile = False
        self.stopFlag = False



def toggleButton(classSettings, attrib, button):
    vars(classSettings)[attrib] = not vars(classSettings)[attrib]
    # print(attrib, vars(classSettings)[attrib])

    if vars(classSettings)[attrib]:
        button.config(bg = 'green')
    else:
        button.config(bg = 'red')


def stopFunction(classSettings, stopButton, startButton):
    classSettings.stopFlag = True
    stopButton.config(bg = 'red')
    startButton.config(state = 'disabled')


def resetFunction(classSettings, button, startButton):
    classSettings.stopFlag = False
    button.config(bg = '#1c1c1c')
    startButton.config(state = 'normal')


def onClosing(classSettings, dictButtons):
    # Exit control loop properly
    classSettings.stopFlag = True
    dictButtons['stopButton'].config(bg = 'red')
    dictButtons['moveButton'].config(state = 'disabled')
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        rootWindow.destroy()


def activateButtons(dictButtons, stopFlag):
    # Reactivate selection buttons 
    for b in dictButtons:
        dictButtons[b].config(state = 'normal')
    if stopFlag:
        dictButtons['moveButton'].config(state = 'disabled')
    dictButtons['moveButton'].config(bg = '#1c1c1c')

def deactivateButtons(dictButtons):
    for b in dictButtons:
        if b != "stopButton":
            dictButtons[b].config(state = 'disabled')


def mapRange(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def updatePressures(dictPress, listPress, minPress, maxPress):
    pIndex = 0
    listIndex = 0
    for p in dictPress:
        if "pressBar" in p:
            # Change colour
            if listPress[pIndex] < 0:
                # Change height of bar
                rectHeight = mapRange(listPress[pIndex], minPress, maxPress, 0, dictPress["canvasHeight"])
                x0, y0, x1, y1 = dictPress["pressCanvas"].coords(dictPress[p])
                dictPress["pressCanvas"].coords(dictPress[p], x0, int(dictPress["canvasHeight"]*guiPressFactor), x1, dictPress["canvasHeight"] - rectHeight)
                # Change colour
                hexInt = int(mapRange(listPress[pIndex], minPress, 0, 255, 0))
                if (hexInt > 255): hexInt = 255
                if (hexInt < 0): hexInt = 0
                barColour = "#0000" + "{0:02x}".format(hexInt)
 
                
            else:
                x0, y0, x1, y1 = dictPress["pressCanvas"].coords(dictPress[p])
                # Change height of bar
                if "Air" in p:
                    rectHeight = mapRange(listPress[pIndex], minPress*4, maxPress*4, 0, dictPress["canvasHeight"])           
                    #Change colour
                    hexInt = int(mapRange(listPress[pIndex], minPress*4, maxPress*4, 0, 255))
                else:
                    rectHeight = mapRange(listPress[pIndex], minPress, maxPress, 0, dictPress["canvasHeight"])
                    #Change colour
                    hexInt = int(mapRange(listPress[pIndex], minPress, maxPress, 0, 255))

                dictPress["pressCanvas"].coords(dictPress[p], x0, dictPress["canvasHeight"] - rectHeight, x1, int(dictPress["canvasHeight"]*guiPressFactor))

                if (hexInt > 255): hexInt = 255
                if (hexInt < 0): hexInt = 0
                barColour = "#" + "{0:02x}".format(hexInt) + "0000"
            # Set colour
            dictPress["pressCanvas"].itemconfig(dictPress[p], fill = barColour)
            pIndex = pIndex + 1

        elif "pressure" in p:
            dictPress[p].config(text = str(listPress[listIndex]))
            listIndex = listIndex + 1



########################################################################################################
# GUI
########################################################################################################

rootWindow = Tk()
rootWindow.title("Soft Robot Control System")
rootWindow.geometry("1000x500")

contentFrame = ttk.Frame(rootWindow)
winStyle = ttk.Style()
winStyle.theme_use("classic")

settingsClass = controlSettings()


# Headings
headingSLabel = Label(contentFrame, text = "Settings", font='bold')
headingLLabel = Label(contentFrame, text = "Status", font='bold')
headingPLabel = Label(contentFrame, text = "Pressures", font='bold')


# Create buttons

buttonDict = {}
calibrateButton = Button(contentFrame, text = "Calibrate robot at start")
attrStr = 'startWithCalibration'
buttonObj = calibrateButton
buttonDict.update({"calibrateButton" : buttonObj})
calibrateButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

optiButton = Button(contentFrame, text = "Use OptiTrack")
attrStr = 'useOptitrack'
buttonObj = optiButton
buttonDict.update({"optiButton" : buttonObj})
optiButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

fibreButton = Button(contentFrame, text = "Use fibrebot")
attrStr = 'useFibrebot'
buttonObj = fibreButton
buttonDict.update({"fibreButton" : buttonObj})
fibreButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

msButton = Button(contentFrame, text = "Use mass spec")
attrStr = 'useMassSpec'
buttonObj = msButton
buttonDict.update({"msButton" : buttonObj})
msButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

omniButton = Button(contentFrame, text = "Use haptic device")
attrStr = 'useOmni'
buttonObj = omniButton
buttonDict.update({"omniButton" : buttonObj})
omniButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')


# pathButton = Button(rootWindow, text = "Follow preprogrammed path")
# attrStr = 'usePathFile'
# buttonObj = optiButton
# buttonDict.update({"optiButton" : buttonObj})
# pathButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
# buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')


visButton = Button(contentFrame, text = "Use pose estimator")
attrStr = 'useVisionFeedback'
buttonObj = visButton
buttonDict.update({"visButton" : buttonObj})
visButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
# visButton.config(command = (lambda s, f, b : toggleButton(s, f, b))(settingsClass, attrStr, buttonObj))



#Start and stop buttons
moveButton = Button(contentFrame, text = "Start robot")
buttonObj = moveButton
buttonDict.update({"moveButton" : buttonObj})

stopButton = Button(contentFrame, text = "Stop")
buttonObj = stopButton
buttonDict.update({"stopButton" : buttonObj})
stopButton.config(command = partial(stopFunction, settingsClass, buttonObj, moveButton))

resetButton = Button(contentFrame, text = "Reset")
buttonObj = stopButton
resetButton.config(command = partial(resetFunction, settingsClass, buttonObj, moveButton))
buttonObj = resetButton
buttonDict.update({"resetButton" : buttonObj})





# Status labels
labelDict = {}
pumpLabel = Label(contentFrame, text = "Pump controller connection")
labelObj = pumpLabel
labelDict.update({"pumpLabel" : labelObj})

omniLabel = Label(contentFrame, text = "Haptic connected")
labelObj = omniLabel
labelDict.update({"omniLabel" : omniLabel})

calibrationLabel = Label(contentFrame, text = "Calibration")
labelObj = calibrationLabel
labelDict.update({"calibrationLabel" : labelObj})






# Progressbars for pressure sensors
pressureDict = {}
barLength = 300
barWidth = 80
barAndPadWidth = 100
padSize = int((barAndPadWidth-barWidth)/2)
numberBars = 4
# Other constants
PRESS_MAX_KPA = 110
VAC_PRESS = -15
guiPressFactor = 1 - abs(VAC_PRESS)/(PRESS_MAX_KPA - VAC_PRESS)

pressureDict.update({"lengthBar" : barLength})

canvasHeight = 200
pressureDict.update({"canvasHeight" : canvasHeight})
pressCanvas = Canvas(contentFrame, width = barAndPadWidth*numberBars, height = canvasHeight, bg='gray')
pressureDict.update({"pressCanvas":pressCanvas})

rectNo = 0
pressureBar1 = pressCanvas.create_rectangle(padSize + rectNo*barAndPadWidth, guiPressFactor*canvasHeight,\
                                            padSize + barWidth + rectNo*barAndPadWidth, guiPressFactor*canvasHeight, fill = 'red')
pressureDict.update({"pressBar1":pressureBar1})

rectNo = 1
pressureBar2 = pressCanvas.create_rectangle(padSize + rectNo*barAndPadWidth, guiPressFactor*canvasHeight,\
                                            padSize + barWidth + rectNo*barAndPadWidth, guiPressFactor*canvasHeight, fill = 'red')
pressureDict.update({"pressBar2":pressureBar2})

rectNo = 2
pressureBar3 = pressCanvas.create_rectangle(padSize + rectNo*barAndPadWidth, guiPressFactor*canvasHeight,\
                                            padSize + barWidth + rectNo*barAndPadWidth, guiPressFactor*canvasHeight, fill = 'red')
pressureDict.update({"pressBar3":pressureBar3})

rectNo = 3
pressureBar4 = pressCanvas.create_rectangle(padSize + rectNo*barAndPadWidth, guiPressFactor*canvasHeight,\
                                            padSize + barWidth + rectNo*barAndPadWidth, guiPressFactor*canvasHeight, fill = 'red')
pressureDict.update({"pressBarAir4":pressureBar4})

pressureL = Label(contentFrame, text = "Pressure L")
pressureR = Label(contentFrame, text = "Pressure R")
pressureT = Label(contentFrame, text = "Pressure T")
pressureStruct = Label(contentFrame, text = "Pressure Struct")
pressureDict.update({"pressureL":pressureL})
pressureDict.update({"pressureR":pressureR})
pressureDict.update({"pressureT":pressureT})
pressureDict.update({"pressureStruct":pressureStruct})


press1Label = Label(contentFrame, text = "Pressure L")
press2Label = Label(contentFrame, text = "Pressure R")
press3Label = Label(contentFrame, text = "Pressure T")
pressPLabel = Label(contentFrame, text = "Pressure Struct")
pressureLabels = [press1Label, press2Label, press3Label, pressPLabel]


# Set command for move Robot button, taking in label dictionary
moveButton.config(command = lambda : threading.Thread(target = moveRobot, args = [buttonDict, labelDict, pressureDict, settingsClass]).start())




#################################################################
# Placement

contentFrame.grid(column=0, row=0)
yPadding = 10
xPadding = 10


headingSLabel.grid(column = 0, row = 0, pady = yPadding, padx = xPadding)
headingLLabel.grid(column = 1, row = 0, pady = yPadding, padx = xPadding)
headingPLabel.grid(column = 4, row = 0, columnspan = 2, pady = yPadding, padx = xPadding)
pressCanvas.grid(column = 3, row = 2, rowspan = 6, columnspan = 4, pady = yPadding, padx = xPadding)

# Place buttons
rowZerothColumn = 1
columnNo = 0
for b in buttonDict:
    if b not in ["moveButton", "stopButton", "resetButton"]:
        buttonDict[b].grid(column = columnNo, row = rowZerothColumn, pady = yPadding, padx = xPadding)
        rowZerothColumn = rowZerothColumn + 1
    else:
        buttonDict[b].grid(column = columnNo, row = rowZerothColumn + 1, pady = yPadding, padx = xPadding)
        columnNo = columnNo + 1
moveButtonRow = rowZerothColumn

# Place labels
rowFirstColumn = 1
columnNo = 1
for l in labelDict:
    labelDict[l].grid(column = columnNo, row = rowFirstColumn, pady = yPadding, padx = xPadding)
    rowFirstColumn = rowFirstColumn + 1


# Place pressure displays and labels
lastRow = max(rowZerothColumn, rowFirstColumn)
columnNo = 3
labelIndex = 0
for pL in pressureLabels:
    pL.grid(column = columnNo, row = 1, pady = yPadding, padx = xPadding)
    columnNo = columnNo + 1
    labelIndex = labelIndex + 1

columnNo = 3
for p in pressureDict:
    if "pressure" in p:
        pressureDict[p].grid(column = columnNo, row = moveButtonRow, pady = yPadding, padx = xPadding)
        columnNo = columnNo + 1



# Set what to do when window is closed
rootWindow.protocol("WM_DELETE_WINDOW", partial(onClosing, settingsClass, buttonDict))

# This is where the magic happens
sv_ttk.set_theme("dark")

#Begin Tk loop
rootWindow.mainloop()

#TODO close threads and disconnect properly if window closed
# make a dict of buttons, not a list
# add status labels e.g. to show if arduinos connected
# add bars that change colour/height depending on pressure (normalse to MAX_PRESS)














