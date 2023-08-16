

import csv
import traceback
import time
import numpy as np 
from tkinter import *
import threading

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
from functools import partial

from visual_navigation.cam_pose import PoseEstimator

np.set_printoptions(suppress=True, precision = 2)




useVisionFeedback = False
visionFeedFlag = False
startWithCalibration = False
useOmni = True
useOptitrack = False
useFibrebot = False
useMassSpec = False
usePathFile = False




CALIBRATION_MODE = 0
HOLD_MODE = 1
ACTIVE_MODE = 2
INFLATION_MODE = 0
DEFLATION_MODE = 1
SET_PRESS_MODE = 3

# Other constants
PRESS_MAX_KPA = 80

CLOSEMESSAGE = "Closed"





######################################################################
def moveRobot(listButtons, classSettings):
    print("'Move robot' button pressed")
    #TODO Deactivate the settings buttons
    for b in listButtons[0:-1]: #Don't disable the stop button
        b.config(state = 'disabled')
    
    [useVisionFeedback, visionFeedFlag, startWithCalibration, useOmni, useOptitrack, useFibrebot, useMassSpec, usePathFile, flagStop] = list(vars(classSettings).values())
    print("Settings: ", vars(classSettings))


    ############################################################
    # Instantiate classes:
    sideLength = 18.78 # mm, from workspace2 model

    kineSolve = kinematics.kineSolver(sideLength)
    # mouseTrack = mouseGUI.mouseTracker(sideLength)
    ardLogging = pumpLog.ardLogger()
    posLogging = positionInput.posLogger()
    opTrack = optiStream.optiTracker()
    phntmOmni = omniStream.omniStreamer()
    dataClust = clusterData.dataClustering()
    pressDetector = arduinoInterface.ardInterfacer
    pumpController = threadArdComms.ardThreader()

    SAMP_FREQ = 1/kineSolve.TIMESTEP


    calibrated = not startWithCalibration
    pumpDataUpdated = False
    delayFactor = 1
    firstMoveDelay = 0
    firstMoveDivider = 100
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
    regulatorPressure = 100
    regulatorSensor = 0
    

    # Use different methods for different paths
    xPath = []
    yPath = []
    zPath = []

    omni_connected = False
    if useOmni:
        omni_connected = phntmOmni.connectOmni()
        print("Haptic device connected? ", omni_connected)

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

    [targetL, targetR, targetT, targetP] = 28.86, 28.86, 28.86, 5  # Centre of scaffold triangle side size 50 mm

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

    if pumpsConnected:
        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, DEFLATION_MODE)
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


    if startWithCalibration:

        if pumpsConnected:

            #  Inflate structure and give some time to stabilise:
            print("Inflating structure...")
            pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, INFLATION_MODE)
            # time.sleep(3)


            # Has the mechanism been calibrated/want to run without calibration?:

            calibrated = False


            # Perform calibration:
            print("Zeroing hydraulic actuators...")
            if (not calibrated):
                pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, INFLATION_MODE)
            while (not calibrated):

                time.sleep(0.007)
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL = pumpController.getData()
                [timeL, timeR, timeT, timeP] = [timeL]*4
                print(pumpController.calibrationFlag)
                print(pressL, pressR, pressT, regulatorSensor, "\n")

                if (pumpController.calibrationFlag == 'Y'):
                    calibrated = True
                    # Send 0s instead of desiredTheta and pressMed as signal that calibration done
                    desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, StepNoA = 0, 0, 0, 0, 0
                    pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0

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
    # if fibreConnected: fibrebotLink.sendState("Run")



    try:
        # if startWithCalibration and not calibrated:
        #     listButtons[-2].config(bg = '#A877BA')
            # raise Exception

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
                if (omniDataReceived == 2): break
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

            # ###[targetL, targetR, targetT, targetOpP] = 28.86, 28.86, 28.86, 5 # FOR HOMING ONLY
            # ###[targetL, targetR, targetT, targetOpP] = 23.09, 23.09, 23.09, 5 # FOR HOMING ONLY23.09
            # [targetL, targetR, targetT, targetOpP] = 10.84, 10.84, 10.84, 5 # FOR HOMING ONLY23.09
            # ###[targetL, targetR, targetT, targetOpP] = 36.32, 16.60, 36.85, 5 # Random non-homing position
            if not omni_connected: 
                [targetL, targetR, targetT, targetOpP] = 10.84, 10.84, 10.84, 5

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

            if pumpsConnected and startWithCalibration:
            # Reduce speed when making first move after calibration.
                if firstMoveDelay < firstMoveDivider:
                    firstMoveDelay += 1
                    # RStep = dStepR scaled for speed (w rounding differences)
                    initStepNoL = int(desiredThetaL*(firstMoveDelay/firstMoveDivider))
                    initStepNoR = int(desiredThetaR*(firstMoveDelay/firstMoveDivider))
                    initStepNoT = int(desiredThetaT*(firstMoveDelay/firstMoveDivider))
                    initStepNoP = int(desiredThetaP*(firstMoveDelay/firstMoveDivider))
                    # Send scaled step number to arduinos:
                    pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, initStepNoP, regulatorPressure, ACTIVE_MODE, INFLATION_MODE)
                else:
                    # Send step number to arduinos:
                    pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, ACTIVE_MODE, INFLATION_MODE)

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


            if pumpsConnected:
                if calibrated:
                    pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, HOLD_MODE, DEFLATION_MODE)
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
        
        finally:
            #Reactivate selection buttons 
            for b in listButtons:
                b.config(state = 'normal')
            listButtons[-2].config(bg = 'white')



###################################################################################################
###################################################################################################


class controlSettings:
    def __init__(self):
        self.useVisionFeedback = False
        self.visionFeedFlag = False
        self.startWithCalibration = True
        self.useOmni = False
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


def stopFunction(classSettings, button):
    classSettings.stopFlag = True
    button.config(bg = 'red')


########################################################################################################
# GUI
########################################################################################################

rootWindow = Tk()
rootWindow.title("Soft Robot Control System")
rootWindow.geometry("700x700")

settingsClass = controlSettings()


# Create buttons
buttonList = []
calibrateButton = Button(rootWindow, text = "Calibrate robot")
attrStr = 'startWithCalibration'
buttonObj = calibrateButton
calibrateButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
buttonList.append(calibrateButton)

optiButton = Button(rootWindow, text = "Use OptiTrack")
attrStr = 'useOptitrack'
buttonObj = optiButton
optiButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
buttonList.append(optiButton)

fibreButton = Button(rootWindow, text = "Use fibrebot")
attrStr = 'useFibrebot'
buttonObj = fibreButton
fibreButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
buttonList.append(fibreButton)

msButton = Button(rootWindow, text = "Use mass spec")
attrStr = 'useMassSpec'
buttonObj = msButton
msButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
buttonList.append(msButton)

omniButton = Button(rootWindow, text = "Use haptic device")
attrStr = 'useOmni'
buttonObj = omniButton
omniButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
buttonList.append(omniButton)

# pathButton = Button(rootWindow, text = "Follow preprogrammed path")
# attrStr = 'usePathFile'
# buttonObj = optiButton
# pathButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
# buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
# buttonList.append(pathButton)

visButton = Button(rootWindow, text = "Use pose estimator")
attrStr = 'useVisionFeedback'
buttonObj = visButton
visButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')
# visButton.config(command = (lambda s, f, b : toggleButton(s, f, b))(settingsClass, attrStr, buttonObj))
buttonList.append(visButton)


#Start and stop buttons
moveButton = Button(rootWindow, text = "Start robot")
buttonList.append(moveButton)

stopButton = Button(rootWindow, text = "Stop")
buttonObj = stopButton
buttonList.append(stopButton)

moveButton.config(command = lambda : threading.Thread(target = moveRobot, args = [buttonList, settingsClass]).start())
stopButton.config(command = partial(stopFunction, settingsClass, buttonObj))


# Place buttons
calibrateButton.pack(pady=20)
omniButton.pack(pady=10)
optiButton.pack(pady=10)
fibreButton.pack(pady=10)
msButton.pack(pady=10)
# pathButton.pack(pady=10)
visButton.pack(pady=10)
moveButton.pack(pady=20)
stopButton.pack(pady=20)


#Begin Tk loop
rootWindow.mainloop()

#TODO close threads etc if window closed
# add status labels e.g. to show if arduinos connected
# add bars that change colour/height depending on pressure (normalse to MAX_PRESS)