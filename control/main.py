

import csv
import traceback
import time
import numpy as np 
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import threading
from functools import partial
import threading
import sv_ttk

np.set_printoptions(suppress=True, precision = 2)

from modules import arduinoInterface
from modules import fibrebotInterface
from modules import massSpecInterface
from modules import kinematics
# from modules import mouseGUI
from modules import ps4_pyUSB
from modules import pumpLog
from modules import positionInput
from modules import optiStream
from modules import omniStream
from modules import clusterData
from modules import threadArdComms
from modules import mouseGUI
from visual_navigation.cam_pose import PoseEstimator



######################################################################
def moveRobot(dictButtons, dictLabel, dictPress, classSettings):
    print("'Move robot' button pressed")

    minPress = VAC_PRESS
    deactivateButtons(dictButtons)
    
    [useVisionFeedback, visionFeedFlag, startWithCalibration, useOmni, socketOmni, useOptitrack, useFibrebot, useMassSpec, usePathFile, goHome, flagStop]\
        = list(vars(classSettings).values())
    
    print("Settings: ", vars(classSettings))


    ############################################################
    # Instantiate classes:
    # sideLength = 18.78 # mm, from workspace2 model
    sideLength = 34 # mm, from workspace2 model

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
    mouseTrack = mouseGUI.mouseTracker(sideLength, kineSolve.MIN_CABLE + kineSolve.RAD_END, kineSolve.MAX_CABLE_DIST)

    SAMP_FREQ = 1/kineSolve.TIMESTEP
    CALIBRATION_MODE = 0
    HOLD_MODE = 1
    ACTIVE_MODE = 2
    ISOLATE_P_SUPPLY = 0
    DEFLATION_MODE = 1
    SET_PRESS_MODE = 3

    HOMING_POSITION = [0, 0, kineSolve.SHAFT_LENGTH_UJ + kineSolve.LEVER_BASE_Z]

    # Other constants
    # PRESS_MAX_KPA = 900
    # VAC_PRESS = -15

    CLOSEMESSAGE = "Closed"


    calibrated = not startWithCalibration
    pumpDataUpdated = False
    delayFactor = 1
    firstMoveDelay = 0
    firstMoveDivider = 200
    delayCount = 0
    delayLim = 200
    reflateFlag = 0

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
    insideBounds = False



    ############################################################################
    # Initialise variables 

    flagStop = False

    # Desired pressure in pneumatic structure
    regulatorPressure = 0
    inflationPressure = 100 #kPa
    regulatorSensor = 0
    

    # Use different methods for different paths
    xPath = []
    yPath = []
    zPath = []

    # Try to connect to both omni and ps4 controller
    # If useOmni flag is set, check for connection and try to use omni. 
    # If not, check for ps4 connection and try to use ps4 controller.
    # If neither omni nor ps4 controller connnected, stay at home position or use mouse 
    if socketOmni is None:
        # If it's running for the first time, est flag to false
        omni_connected = False
    else: 
        # If the omni has already been connected, use existing settings
        omni_connected = True
        phntmOmni.sock = classSettings.socketOmni

    omni_connected = phntmOmni.connectOmni(omni_connected)
    print("Haptic device connected? ", omni_connected)
    # omni_connected = False

    ps4 = ps4_pyUSB.ps4USB() # Create an object from controller
    if ps4.controller is not None:
        ps4.start() #start and listen to events
        xPS4, yPS4, zPS4 = 0, 0, 0
        print("PS4 Controller connected")
        ps4Buttons = 0
    
    if omni_connected:
        if ps4.controller is not None:
            dictLabel["omniLabel"].config(fg = "green")
        else:
            dictLabel["omniLabel"].config(fg = "magenta")
    else:
        if ps4.controller is not None:
            dictLabel["omniLabel"].config(fg = "cyan")
        else:
            dictLabel["omniLabel"].config(fg = "red")
    
    omniButtons = 0 #phntmOmni.omniButton # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both

    # Try to connect to phantom omni. If not connected, use pre-determined coords.
    # if usePathFile:
    if useOmni:
        if omni_connected:
            # omniX, omniY, omniZ = 0.0, 0.0, 0.0
            omniDataReceived = phntmOmni.getOmniCoords()
            [xMap, yMap, zMap] = phntmOmni.omniMap()
            
        else:
            with open('C:/Users/msrun/OneDrive - Imperial College London/Imperial/DataLogs/DT_Prime/paths/gridPath 2023-03-03 16-29-08 centre 15-8.66025 30x15.0grid 0.048x1.5spacing.csv', newline = '') as csvPath:
                coordReader = csv.reader(csvPath)
                for row in coordReader:
                    xPath.append(float(row[0]))
                    yPath.append(float(row[1]))
                    zPath.append(float(row[2]))
                xMap, yMap, zMap = xPath[0], yPath[0], zPath[0]
            xMap, yMap, zMap = HOMING_POSITION[0], HOMING_POSITION[1], kineSolve.SHAFT_LENGTH_UJ + kineSolve.LEVER_BASE_Z

    else:
        # if ps4.controller is not None:
        xMap, yMap, zMap = HOMING_POSITION[0], HOMING_POSITION[1], kineSolve.SHAFT_LENGTH_UJ + kineSolve.LEVER_BASE_Z

    # Button setting from controller for grasper control
    controllerButtons = 0

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
    pressList = [pressL, pressR, pressT, regulatorSensor]
    prevPressL, prevPressR, prevPressT, prevPressP = 0, 0, 0, 0
    pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0
    timeL, timeR, timeT, timeP = 0, 0, 0, 0
    prevTimeL, prevTimeR, prevTimeT, prevTimeP = 0, 0, 0, 0
    conLHS, conRHS, conTOP = 0, 0, 0
    dLHS, dRHS, dTOP = 0, 0, 0
    collisionAngle = 1j

    # Load cell data
    loadL, loadR, loadT, loadP = 0, 0, 0, 0
    loadList = [loadL, loadR, loadT, loadP]

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
        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, ISOLATE_P_SUPPLY, controllerButtons)
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
    msCOM = 'COMX'
    msConnected = False
    if useMassSpec:
        msConnected = massSpecLink.connect(msCOM)
        # if msConnected:
        print("Mass spec serial connected? ", msConnected)


    ###############################################################################################
    # The most important try statement
    try:

        if pumpsConnected:
            dictPress["pressureL"].config(fg = "white")
            dictPress["pressureR"].config(fg = "white")
            dictPress["pressureT"].config(fg = "white")
            dictPress["pressureStruct"].config(fg = "white")
            time.sleep(1.5)
            if not messagebox.askokcancel("Inflate structure?", "Would you like to inflate the structure?"):
                raise
            #  Inflate structure and give some time to stabilise:
            print("Inflating structure...")
            pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, controllerButtons)
            count = 0
            countLimit = 50
            rampTime = 5 # seconds
            while (count <= countLimit):
                regulatorPressure = round(inflationPressure*(count/countLimit))
                # print(regulatorPressure)

                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()
                time.sleep(rampTime/countLimit)

                pressList = [pressL, pressR, pressT, regulatorSensor]
                # Change pressurebars
                updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)
                count = count + 1

                # Stop operation if Stop button hit
                flagStop = classSettings.stopFlag
                if flagStop == True:
                    activateButtons(dictButtons, flagStop)
                    raise 

                ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, loadL, timeL)
                ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, loadR, timeR)
                ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, loadT, timeT)
                ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, loadP, timeP)
                # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
                # Ensure same number of rows in position log file
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

                pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, controllerButtons)
            time.sleep(0.07)
            pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, controllerButtons)

            # Wait an additional 3 s to stabilise

            if not messagebox.askokcancel("Proceed?", "Was inflation succesfull?"):
                raise

            if startWithCalibration:
                
                # Has the mechanism been calibrated/want to run without calibration?:
                calibrated = not startWithCalibration



                if (not calibrated):
                    # Perform calibration:
                    print("Zeroing hydraulic actuators...")
                    dictLabel["calibrationLabel"].config(fg = "red")
                    pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, SET_PRESS_MODE, controllerButtons)

                prevCaliState = pumpController.calibrationByte
                    
                while (not calibrated):

                    # Stop operation if Stop button hit
                    flagStop = classSettings.stopFlag
                    if flagStop == True:
                        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, controllerButtons)
                        time.sleep(0.1)
                        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, HOLD_MODE, SET_PRESS_MODE, controllerButtons)
                        activateButtons(dictButtons, flagStop)
                        raise

                    pressLMed = medPressL.newPressMed(pressL)
                    pressRMed = medPressR.newPressMed(pressR)
                    pressTMed = medPressT.newPressMed(pressT)
                    pressP = regulatorSensor
                    pressPMed = medPressP.newPressMed(regulatorSensor)
                    if (max(pressLMed, pressRMed, pressTMed) > PRESS_MAX_KPA): # TODO Add filtered pressure values back again to use here
                        print("Overpressure: ", max(pressL, pressR, pressT), " kPa")
                        raise

                    [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()
                    [timeL, timeR, timeT, timeP] = [timeL]*4
                    if prevCaliState != pumpController.calibrationByte:
                        prevCaliState = pumpController.calibrationByte
                    # print(int.from_bytes(pumpController.calibrationByte,'little'), pumpController.calibrationFlag)
                    if (int.from_bytes(pumpController.calibrationByte,'little') & 1 != 0):
                        dictPress["pressureL"].config(fg = "green")
                    if (int.from_bytes(pumpController.calibrationByte,'little') & 2 != 0):
                        dictPress["pressureR"].config(fg = "green")
                    if (int.from_bytes(pumpController.calibrationByte,'little') & 4 != 0):
                        dictPress["pressureT"].config(fg = "green")
                    if (int.from_bytes(pumpController.calibrationByte,'little') & 8 != 0):
                        dictPress["pressureStruct"].config(fg = "cyan")
                    # print(pressL, pressR, pressT, regulatorSensor, "\n")

                    pressList = [pressLMed, pressRMed, pressTMed, pressPMed]
                    # Change pressurebars
                    updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)

                    if (pumpController.calibrationFlag == 'Y'):
                        dictLabel["calibrationLabel"].config(fg = "green")
                        calibrated = True
                        # Send 0s instead of desiredTheta and pressMed as signal that calibration done
                        desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, StepNoA = 0, 0, 0, 0, 0
                        pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0, 0, 0, 0, 0
                    else:
                        time.sleep(0.009)
                        pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, StepNoP, regulatorPressure, CALIBRATION_MODE, SET_PRESS_MODE, controllerButtons)

                    ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, loadL, timeL)
                    ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, loadR, timeR)
                    ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, loadT, timeT)
                    ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, loadP, timeP)
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

        if not messagebox.askokcancel("Proceed?", "Start the robot? MANUAL CALIBRATION COMPLETE?"):
            raise

        ################################################################
        # Begin main loop
        mouseTrack.createTracker(kineSolve.attach_points_rot)
        dictPress["pressureL"].config(fg = "white")
        dictPress["pressureR"].config(fg = "white")
        dictPress["pressureT"].config(fg = "white")
        dictPress["pressureStruct"].config(fg = "white")
        while(flagStop == False):

            useOmni = classSettings.useOmni
            if useOmni:
                if omni_connected:
                    omniDataReceived = phntmOmni.getOmniCoords()
                    omniButtons = phntmOmni.omniButton
                    controllerButtons = omniButtons
                    frameRotAngle = dictLabel["rotationSlider"].get()
                    if (omniDataReceived == 2): break
                    [xMap, yMap, zMap] = phntmOmni.omniMap(frameRotAngle)
                    XYZPathCoords = [xMap, yMap, zMap]
                else:
                    controllerButtons = 0
                    XYZPathCoords = [HOMING_POSITION[0], HOMING_POSITION[1], XYZPathCoords[2]]
                    # print(XYZPathCoords)
            else:
                if ps4.controller is not None:
                    ps4Buttons = ps4.getPSButtonData()
                    if ps4Buttons == 3:
                        regulatorPressure = inflationPressure + 1 if reflateFlag else inflationPressure
                        reflateFlag = 0 if reflateFlag else 1
                        # print(regulatorPressure, reflateFlag)
                    controllerButtons = ps4Buttons
                    frameRotAngle = dictLabel["rotationSlider"].get()
                    prevxPS4 = xPS4
                    prevyPS4 = yPS4
                    prevzPS4 = zPS4
                    [temp_xPS4, temp_yPS4, temp_zPS4] = ps4.incrementXYZCoords(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], frameRotAngle)
                    # Prevent motion if going out of reachable workspace:
                    [targetXideal, targetYideal, targetOpP, inclin, azimuth] = kineSolve.intersect(temp_xPS4, temp_yPS4, temp_zPS4)
                    [targetOpL, targetOpR, targetOpT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetXideal, targetYideal)
                    if (targetOpL >= kineSolve.MAX_CABLE_DIST) or (targetOpR >= kineSolve.MAX_CABLE_DIST) or (targetOpT >= kineSolve.MAX_CABLE_DIST):
                        xPS4, yPS4, zPS4 = prevxPS4, prevyPS4, prevzPS4
                    else:
                        xPS4, yPS4, zPS4 = temp_xPS4, temp_yPS4, temp_zPS4
                    # Check limits on prismatic
                    if (targetOpP >= kineSolve.MAX_EXTEND) and (zPS4 >= prevzPS4):
                        zPS4 = prevzPS4
                    elif (targetOpP <= kineSolve.MIN_EXTEND) and (zPS4 <= prevzPS4):
                        zPS4 = prevzPS4
                    # if abs(targetXideal) > kineSolve.SIDE_LENGTH/2:
                    #     xPS4 = prevxPS4
                    # if abs(targetYideal) > kineSolve.SIDE_LENGTH/2:
                    #     yPS4 = prevyPS4
                    XYZPathCoords = [xPS4, yPS4, zPS4]
                    # print(XYZPathCoords)
                # elif pathCounter >= len(xPath):
                #     break               
                else: #Nothing is connected, stay at home position
                    controllerButtons = 0
                    XYZPathCoords = [HOMING_POSITION[0], HOMING_POSITION[1], XYZPathCoords[2]]

            if classSettings.goToHome:
                controllerButtons = 0
                XYZPathCoords = [HOMING_POSITION[0], HOMING_POSITION[1], XYZPathCoords[2]]


            # Ideal target points refer to non-discretised coords on parallel mechanism plane, otherwise, they are discretised.
            # XYZPathCoords are desired coords in 3D.
            [targetXideal, targetYideal, targetOpP, inclin, azimuth] = kineSolve.intersect(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2])
            POIcoords = [targetXideal, targetYideal]
            # If no controller connected, set POICoords to None to make mouse control possible
            if (omni_connected == False) and (ps4.controller is None):
                POIcoords = None
            [targetX_mouse, targetY_mouse, flagStop, insideBounds] = mouseTrack.iterateTracker(loadList, kineSolve.attach_points_rot, POIcoords, XYZPathCoords)

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
            # print("Target cable lengths: ", targetL, targetR, targetT, targetP)
            # print(lhsV, rhsV, topV, actualX, actualY)
            # print(cVolL, cVolR, cVolT)

            # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
            # Use current volume, current cable length and target cable length
            [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
            [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
            [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
            # print("\n",tStepL, tStepR, tStepT, "\n")
            # print("\nL_cs in mm : ", LcRealL, LcRealR, LcRealT, targetP)
            # print("Volumes in ml: ",tVolL/1000, tVolR/1000, tVolT/1000, targetOpP, "\n")

            [tVolL_Scaled, tVolR_Scaled, tVolT_Scaled] = kineSolve.volRateScale(tVolL, tVolR, tVolT, cVolL, cVolR, cVolT)

            desiredThetaL = kineSolve.volToAngle(tVolL_Scaled)
            desiredThetaR = kineSolve.volToAngle(tVolR_Scaled)
            desiredThetaT = kineSolve.volToAngle(tVolT_Scaled)
            desiredThetaP = 360.0*targetOpP/kineSolve.LEAD
            # print(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, "\n")

            # Log desired positions
            if pumpDataUpdated:
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

            if pumpsConnected:
                # if startWithCalibration:
                #     # Reduce speed when making first move after calibration.
                # if useOmni:
                #     if omni_connected:
                #         controllerButtons = omniButtons
                #     else:
                #         controllerButtons = 0
                # else:
                #     if ps4.controller is not None:
                #         controllerButtons = ps4Buttons
                #     else:
                #         controllerButtons = 0
                # print(controllerButtons)    
                
                # regulatorPressure = inflationPressure

                if firstMoveDelay < firstMoveDivider:
                    firstMoveDelay += 1
                    # RStep = dStepR scaled for speed (w rounding differences)
                    initStepNoL = int(desiredThetaL*(firstMoveDelay/firstMoveDivider))
                    initStepNoR = int(desiredThetaR*(firstMoveDelay/firstMoveDivider))
                    initStepNoT = int(desiredThetaT*(firstMoveDelay/firstMoveDivider))
                    initStepNoP = int(desiredThetaP*(firstMoveDelay/firstMoveDivider))
                    # Send scaled step number to arduinos:
                    pumpController.sendStep(initStepNoL, initStepNoR, initStepNoT, initStepNoP, regulatorPressure, ACTIVE_MODE, SET_PRESS_MODE, controllerButtons)
                else:
                    # Send step number to arduinos:
                    pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, ACTIVE_MODE, SET_PRESS_MODE, controllerButtons)

                # Log values from arduinos
                if pumpDataUpdated:
                    ardLogging.ardLog(realStepL, LcRealL, targetL, desiredThetaL, pressL, pressLMed, loadL, timeL)
                    ardLogging.ardLog(realStepR, LcRealR, targetR, desiredThetaR, pressR, pressRMed, loadR, timeR)
                    ardLogging.ardLog(realStepT, LcRealT, targetT, desiredThetaT, pressT, pressTMed, loadT, timeT)
                    ardLogging.ardLog(realStepP, LcRealP, targetP, desiredThetaP, pressP, pressPMed, loadP, timeP)
                    # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                    ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)

                # Get current pump position, pressure and times from arduinos
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()
                [timeL, timeR, timeT, timeP] = [timeL]*4

                pressLMed = medPressL.newPressMed(pressL)
                pressRMed = medPressR.newPressMed(pressR)
                pressTMed = medPressT.newPressMed(pressT)
                pressP = regulatorSensor
                pressPMed = medPressP.newPressMed(regulatorSensor)

                # pressList = [pressL, pressR, pressT, regulatorSensor]
                pressList = [pressLMed, pressRMed, pressTMed, pressPMed]
                loadList = [loadL, loadR, loadT, loadP]
 
                # Change pressurebars
                updatePressures(dictPress, pressList, minPress, PRESS_MAX_KPA)

                # Check for high pressure
                if (max(pressLMed, pressRMed, pressTMed) > 85): # TODO Add filtered pressure values back again to use here
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
                pumpDataUpdated = True
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
            pathCounter += 1
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
                ardLogging.ardLog(realStepL, LcRealL, angleL, desiredThetaL, pressL, pressLMed, loadL, timeL)
                ardLogging.ardLog(realStepR, LcRealR, angleR, desiredThetaR, pressR, pressRMed, loadR, timeR)
                ardLogging.ardLog(realStepT, LcRealT, angleT, desiredThetaT, pressT, pressTMed, loadT, timeT)
                ardLogging.ardLog(realStepP, LcRealP, angleP, desiredThetaP, pressP, pressPMed, loadP, timeP)
                # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
                ardLogging.ardSave()
                # Ensure same number of rows in position log file
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, azimuth)

                # if calibrated:
                controllerButtons = 0
                pumpController.sendStep(desiredThetaL, desiredThetaR, desiredThetaT, desiredThetaP, regulatorPressure, HOLD_MODE, DEFLATION_MODE, controllerButtons)
                pumpController.stopThreader()
                time.sleep(0.2)
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()
                [timeL, timeR, timeT, timeP] = [timeL]*4
                print("Connection to pump controller closed.")
                print(realStepL, pressL, timeL)
                print(realStepR, pressR, timeR)
                print(realStepT, pressT, timeT)
                print(realStepP, pressP, timeP)
                pumpController.t.stop()
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
                # try:
                phntmOmni.omniClose()
                phntmOmni.omniServer.kill()
                omni_connected = False
                classSettings.socketOmni = phntmOmni.sock
                # except SocketError:

            if ps4.controller is not None:
                ps4.stop_ps4()



            mouseTrack.closeTracker()

        except TypeError as exTE:
            tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
            tb_textTE = ''.join(tb_linesTE)
            print(tb_textTE)

        dictPress["pressureL"].config(fg = "white")
        dictPress["pressureR"].config(fg = "white")
        dictPress["pressureT"].config(fg = "white")
        dictPress["pressureStruct"].config(fg = "white")
        
        
        print("Move Robot complete.")





###################################################################################################
###################################################################################################


class controlSettings:
    def __init__(self):
        self.useVisionFeedback = False
        self.visionFeedFlag = False
        self.startWithCalibration = True
        self.useOmni = False
        self.socketOmni = None
        self.useOptitrack = False
        self.useFibrebot = False
        self.useMassSpec = False
        self.usePathFile = False
        self.goToHome = False
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
        for thread in threading.enumerate(): 
            if thread.name != "MainThread":
            #    thread._connection_made.set()
            #    thread.set
               thread.join()
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
        if b not in ["stopButton", "homeButton", "omniButton"]:
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

homeButton = Button(contentFrame, text = "Home Position")
attrStr = 'goToHome'
buttonObj = homeButton
buttonDict.update({"homeButton" : buttonObj})
homeButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')



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
VAC_PRESS = -40
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


# Creates slider to rotate input device coordinates.
# To be placed below the pressure bars, so it is the same width as the pressure bar canvas
rotationSlider = Scale(contentFrame, from_=180, to=-180, length = barAndPadWidth*numberBars, tickinterval=60, orient = HORIZONTAL)
labelDict.update({"rotationSlider" : rotationSlider})



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
# columnNo = 0
# homeButton.grid(column = columnNo, row = rowZerothColumn + 2, pady = yPadding, padx = xPadding)
moveButtonRow = rowZerothColumn

columnNo = 3
rotationSlider.grid(column = columnNo, row = rowZerothColumn + 2, columnspan = 4, pady = yPadding, padx = xPadding)

# Place labels
rowFirstColumn = 1
columnNo = 1
for l in labelDict:
    if l not in ["rotationSlider"]: # Exclude rotation slider here
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
        pressureDict[p].grid(column = columnNo, row = moveButtonRow - 1, pady = yPadding, padx = xPadding)
        columnNo = columnNo + 1



# Set what to do when window is closed
rootWindow.protocol("WM_DELETE_WINDOW", partial(onClosing, settingsClass, buttonDict))

# This is where the magic happens
sv_ttk.set_theme("dark")

#Begin Tk loop
# rootWindow.attributes('-topmost',True)
rootWindow.mainloop()

#TODO close threads and disconnect properly if window closed
# make a dict of buttons, not a list
# add status labels e.g. to show if arduinos connected
# add bars that change colour/height depending on pressure (normalse to MAX_PRESS)














