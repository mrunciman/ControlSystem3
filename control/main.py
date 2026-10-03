

import csv
import traceback
import time
import numpy as np 
import math as mt
from mttkinter import mtTkinter as tk
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import threading
from functools import partial
import sv_ttk
import multiprocessing

np.set_printoptions(suppress=True, precision = 2)


from modules import kinematics
from modules import ps4_pyUSB
from modules import pumpLog
from modules import positionInput
from modules import optiStream
from modules import threadArdComms
from modules import pandaViewer
from modules import jointOffsets



######################################################################
def moveRobot(dictButtons, dictLabel, classSettings, pumpController, viewerInputList):
    
    print("Motion control started.")

    deactivateButtons(dictButtons)
    
    [jointOffsetButtons, visionFeedFlag, startWithCalibration, useOmni, socketOmni, useOptitrack, useFibrebot, moveRobotRunning, usePathFile, goHome, flagStop, socketFalcon, destroyWindow]\
        = list(vars(classSettings).values())

    classSettings.moveRobotRunning = True
    # print("Settings: ", vars(classSettings))
    classSettings.pandaViewerProcess.start()

    ############################################################
    # Instantiate classes:

    kineSolve = kinematics.kineSolver()
    ardLogging = pumpLog.ardLogger()
    posLogging = positionInput.posLogger()
    opTrack = optiStream.optiTracker()

    SAMP_FREQ = 1/kineSolve.TIMESTEP
    CALIBRATION_MODE = 0
    HOLD_MODE = 1
    ACTIVE_MODE = 2
    ISOLATE_P_SUPPLY = 0
    DEFLATION_MODE = 1
    SET_PRESS_MODE = 3

    HOMING_POSITION = [0, 0, 0]


    pumpDataUpdated = False
    firstMoveDelay = 0
    firstMoveDivider = 400


    ############################################################################
    # Initialise variables 

    flagStop = False

    # Use different methods for different paths
    xPath = []
    yPath = []
    zPath = []

    ps4 = ps4_pyUSB.ps4USB() # Create an object from controller
    if ps4.controller is not None:
        ps4.start() #start and listen to events
        ps4Buttons = 0
        dictLabel["omniLabel"].config(fg = "green")
    else:
        dictLabel["omniLabel"].config(fg = "red")

    print("Single-hand controller connected? ", ps4.controller is not None)
    
    if ps4.controller is not None:
        xMap, yMap, zMap = HOMING_POSITION[0], HOMING_POSITION[1], HOMING_POSITION[2]

    ############################################################
    pathCounter = 0
    if usePathFile:
        with open('C:/Users/msrun/OneDrive - Imperial College London/Imperial/DataLogs/DT_Prime/paths/gridPath 2023-03-03 16-29-08 centre 15-8.66025 30x15.0grid 0.048x1.5spacing.csv', newline = '') as csvPath:
            coordReader = csv.reader(csvPath)
            for row in coordReader:
                xPath.append(float(row[0]))
                yPath.append(float(row[1]))
                zPath.append(float(row[2]))
            xMap, yMap, zMap = xPath[0], yPath[0], zPath[0]

    # Button setting from controller for grasper control
    controllerButtons = 0

    XYZPathCoords = [xMap, yMap, zMap]

    # Target must be cast as immutable type (float, in this case) so that 
    # the current position doesn't update at same time as target
    currentX = XYZPathCoords[0]
    currentY = XYZPathCoords[1]
    currentZ = XYZPathCoords[2]
    targetX = XYZPathCoords[0]
    targetY = XYZPathCoords[1]
    targetZ = XYZPathCoords[2]


    desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos = 0.5, 0, 0, 0, 0
    axialPos, rotaryPos, toolExt, wristAngle, graspPos = 0, 0, 0, 0, 0

    desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp = 0, 0, 0, 0, 0

    # Set initial pressure and calibration variables
    timeL = 0
    prevTimeL = 0

    # Current position
    initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp = 0, 0, 0, 0, 0

    jointOffsetList = [0, 0, 0, 0, 0, 0]

    ############################################################################
    # Visual servoing variables

    ############################################################################
    # Optitrack connection
    useRigidBodies = True
    optiTrackConnected = False
    if useOptitrack:
        optiTrackConnected = opTrack.optiConnect()

    ###############################################################
    # Connect to Peripherals

    # startThreader opens the serial connection and starts the communication thread
    pumpsConnected = pumpController.connected
    print("Connected to Control Unit? ", pumpsConnected)
    
    dictLabel["pumpLabel"].config(fg = "green") if pumpsConnected else dictLabel["pumpLabel"].config(fg = "red")

    if pumpsConnected:
        pumpController.sendStep(initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp, HOLD_MODE, ISOLATE_P_SUPPLY, controllerButtons)



    ###############################################################################################
    # The most important try statement
    try:

        if pumpsConnected:
            time.sleep(1.5)
            if not messagebox.askokcancel("Structure deployed?", "Has the structure been deployed?"):
                raise
            pumpController.sendStep(initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp, HOLD_MODE, SET_PRESS_MODE, controllerButtons)

        else:
            print("PUMP CONTROLLER NOT CONNECTED. RUNNING WITHOUT PUMPS.")


        if not messagebox.askokcancel("Proceed?", "Start the robot? MANUAL CALIBRATION COMPLETE?"):
            raise

        ################################################################
        # Begin main loop

        while(flagStop == False):

            # Get offsest values from GUI
            jointOffsetIndex = 0
            for joint in jointOffsetButtons:
                jointOffsetList[jointOffsetIndex] = joint.offset.get()
                jointOffsetIndex = jointOffsetIndex + 1

                                
            if ps4.controller is not None:
                ps4Buttons = ps4.getPSButtonData()
                controllerButtons = ps4Buttons

                # This gives desired joint values 
                [desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos] = ps4.incrementCylCoords(axialPos, rotaryPos, toolExt, wristAngle, graspPos)
                
                # Convert desired joint values into angular positions of each motor
                intermedAxial, desAxialPos = kineSolve.setAxialMotor(desAxialPos, desWristAngle)
                intermedRotary, desRotaryPos = kineSolve.setRotaryMotor(desRotaryPos, rotaryPos)
                intermedTool, desTooExt = kineSolve.setToolMotor(desTooExt)
                intermedWrist, desWristAngle = kineSolve.setWristMotor(desWristAngle)
                intermedGrasp, desGraspPos = kineSolve.setGraspMotor(desGraspPos)
                # print("Motor angles: ", desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, "\n")
                

                desiredThetaAxial = intermedAxial + jointOffsetList[0]
                desiredThetaRotary = intermedRotary + jointOffsetList[1]
                desiredThetaTool = intermedTool + jointOffsetList[2]
                desiredThetaWrist = intermedWrist + jointOffsetList[3]
                desiredThetaGrasp = intermedGrasp + jointOffsetList[4]


                frameRotAngle = dictLabel["rotationSlider"].get()
                

            if classSettings.goToHome:
                controllerButtons = 0


            if controllerButtons == 1:
                dictLabel["grasperLabel"].config(text = "Grasper open", fg = "green")
            elif controllerButtons == 2:
                dictLabel["grasperLabel"].config(text = "Grasper close", fg = "red")
            elif controllerButtons == 3:
                dictLabel["grasperLabel"].config(text = "Grasper retract", fg = "orange")
            elif controllerButtons == 4:
                dictLabel["grasperLabel"].config(text = "Grasper extend", fg = "cyan")
            else:
                dictLabel["grasperLabel"].config(text = "Grasper", fg = "white")


            visualiseOffset = 1
            viewerInputList[0] = desAxialPos + jointOffsetList[0]*visualiseOffset
            viewerInputList[1] = desRotaryPos + jointOffsetList[1]*visualiseOffset
            viewerInputList[2] = desTooExt + jointOffsetList[2]*visualiseOffset
            viewerInputList[3] = desWristAngle + jointOffsetList[3]*visualiseOffset
            viewerInputList[4] = desGraspPos + jointOffsetList[4]*visualiseOffset
            viewerInputList[5] = 0 + jointOffsetList[5]*visualiseOffset
            # print("From tkinter ", viewerInputList)

    
            # desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp
            # Log desired positions
            if pumpDataUpdated:
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, ang_around_shaft)

            if pumpsConnected:

                if firstMoveDelay < firstMoveDivider:
                    firstMoveDelay += 1
                    # RStep = dStepR scaled for speed (w rounding differences)
                    initThetaAxial = (desiredThetaAxial*(firstMoveDelay/firstMoveDivider))
                    initThetaRot = (desiredThetaRotary*(firstMoveDelay/firstMoveDivider))
                    initThetaTool = (desiredThetaTool*(firstMoveDelay/firstMoveDivider))
                    initThetaWrist = (desiredThetaWrist*(firstMoveDelay/firstMoveDivider))
                    initThetaGrasp = (desiredThetaGrasp*(firstMoveDelay/firstMoveDivider))
                    # Send scaled step number to arduinos:
                    pumpController.sendStep(initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp, ACTIVE_MODE, SET_PRESS_MODE, controllerButtons)
                else:
                    # Send step number to arduinos:
                    pumpController.sendStep(desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, ACTIVE_MODE, SET_PRESS_MODE, controllerButtons)

                # Log values from arduinos
                if pumpDataUpdated:
                    # ardLogging.ardLog(realStepL, LcRealL, targetL, desiredThetaL, pressL, pressLMed, loadL, timeL) 
                    # ardLogging.ardLog(realStepR, LcRealR, targetR, desiredThetaR, pressR, pressRMed, loadR, timeR)
                    # ardLogging.ardLog(realStepT, LcRealT, targetT, desiredThetaT, pressT, pressTMed, loadT, timeT)
                    # ardLogging.ardLog(realStepP, LcRealP, targetP, desiredThetaP, pressP, pressPMed, loadP, timeP)
                    # # ardLogging.ardLog(realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA)
                    ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)#TODO Rewrite as one logging function

                # Get current pump position, pressure and times from arduinos
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()


            # Check if new data has been received from pump controller 
            if (timeL - prevTimeL > 0):
                pumpDataUpdated = True
                # kineSolve.TIMESTEP = (timeL - prevTimeL)/1000
                kineSolve.TIMESTEP = 0.01
                if (kineSolve.TIMESTEP < 0.01): kineSolve.TIMESTEP = 0.01
                # print((timeL - prevTimeL)/1000)
            else:
                # pumpDataUpdated = True
                kineSolve.TIMESTEP = 0.01

            # Update current position, cable lengths, and volumes as previous targets
            prevTimeL = timeL
            prevPathCounter = pathCounter
            pathCounter += 1
            # if pumpDataUpdated: pathCounter += 1
            axialPos, rotaryPos, toolExt, wristAngle, graspPos = desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos

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

        print("Ending loop...")

        ###########################################################################
        # Stop program
        # Disable pumps and set them to idle state
        try:

            if pumpsConnected:
                # Save values gathered from arduinos
                ardLogging.ardLogCollide(conLHS, conRHS, conTOP, collisionAngle)
                # Save joint information
                ardLogging.ardSave()
                # Ensure same number of rows in position log file
                posLogging.posLog(XYZPathCoords[0], XYZPathCoords[1], XYZPathCoords[2], inclin, ang_around_shaft)
                #Save position data
                posLogging.posSave()

                # if calibrated:
                controllerButtons = 0
                pumpController.sendStep(desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, HOLD_MODE, DEFLATION_MODE, controllerButtons)
                time.sleep(0.2)
                n = 20
                for x in range(n):
                    pumpController.sendStep(desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, HOLD_MODE, DEFLATION_MODE, controllerButtons)
                    time.sleep(0.2)
                    # print(x)
                time.sleep(0.2)
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()


            # #Save optitrack data
            if optiTrackConnected:
                if useRigidBodies:
                    opTrack.optiSave(opTrack.rigidData)
                else:
                    opTrack.optiSave(opTrack.markerData)
                opTrack.optiClose()

            # Close controller thread
            if ps4.controller is not None:
                ps4.stop_ps4()


        except TypeError as exTE:
            tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
            tb_textTE = ''.join(tb_linesTE)
            print(tb_textTE)

                
        print("Move Robot complete.")
        classSettings.moveRobotRunning = False





###################################################################################################
###################################################################################################


class controlSettings:
    def __init__(self):
        self.jointOffsetButtons = None
        self.pandaViewerProcess = False
        self.startWithCalibration = False
        self.useOmni = False
        self.socketOmni = None
        self.useOptitrack = False
        self.useFibrebot = False
        self.moveRobotRunning = False
        self.usePathFile = False
        self.goToHome = False
        self.stopFlag = False
        self.socketFalcon = None
        self.destroyWindow = False



def toggleButton(classSettings, attrib, button):
    vars(classSettings)[attrib] = not vars(classSettings)[attrib]
    # print(attrib, vars(classSettings)[attrib])

    if vars(classSettings)[attrib]:
        button.config(bg = 'green')
    else:
        button.config(bg = 'red')


def toggleInputButton(classSettings, attrib, button):
    inputSelect = vars(classSettings)[attrib]
    newInputSelect = (inputSelect + 1) % 3
    vars(classSettings)[attrib] = newInputSelect
    # print(attrib, vars(classSettings)[attrib])

    # Use ps4 controller
    if newInputSelect == 0:
        button.config(text = "PS4", bg = 'red')
    #Use phantom omni
    elif newInputSelect == 1:
        button.config(text = "Omni", bg = 'green')
    # Usee falcon controller
    elif newInputSelect == 2:
        button.config(text = "Falcon", bg = 'blue')


def stopFunction(classSettings, stopButton, startButton, viewerProcess):
    classSettings.stopFlag = True
    stopButton.config(bg = 'red')
    startButton.config(state = 'disabled')

    if viewerProcess.is_alive():
        viewerProcess.kill()
    classSettings.pandaViewerProcess = multiprocessing.Process(target=viewer_process, args=(viewerInput,))
    classSettings.pandaViewerProcess.daemon = True


def resetFunction(classSettings, button, startButton):
    classSettings.stopFlag = False
    button.config(bg = '#1c1c1c')
    startButton.config(state = 'normal')


def onClosing(classSettings, dictButtons, viewerProcess):
    # Exit control loop properly
    classSettings.stopFlag = True
    dictButtons['stopButton'].config(bg = 'red')
    dictButtons['moveButton'].config(state = 'disabled')
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        for thread in threading.enumerate():
            if thread.name != "MainThread":
                if thread.name == "ardThread":
                   pumpController.stopThreader()
                   pumpController.t.stop()
                   pumpController.closeSerial()
                else:
                    exitCode = thread.join()
                    print(exitCode, thread.is_alive())

        classSettings.destroyWindow = True
        if viewerProcess.is_alive():
            viewerProcess.kill()


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


def viewer_process(angleList):
    pandaViewer.run_viewer(angleList)

########################################################################################################
# GUI
########################################################################################################
if __name__ == '__main__':
    rootWindow = Tk()
    rootWindow.title("Lannsair Endoluminal Robotics")
    rootWindow.geometry("800x400")

    contentFrame = ttk.Frame(rootWindow)
    winStyle = ttk.Style()
    winStyle.theme_use("classic")

    settingsClass = controlSettings()

    manager = multiprocessing.Manager()

    initialAngle1, initialAngle2, prismShaft = 0, 0, 0
    shaftStartX, shaftStartY, shaftStartZ = 0, 0, 0,
    viewerInput = manager.list([initialAngle1, initialAngle2, prismShaft, shaftStartX, shaftStartY, shaftStartZ])
    pandaViewerProcess = multiprocessing.Process(target=viewer_process, args=(viewerInput,))
    pandaViewerProcess.daemon = True
    settingsClass.pandaViewerProcess = pandaViewerProcess

    # Headings
    headingSLabel = Label(contentFrame, text = "Settings", font='bold')
    headingLLabel = Label(contentFrame, text = "Status", font='bold')
    headingPLabel = Label(contentFrame, text = "Offsets", font='bold')


    # Create buttons

    buttonDict = {}
    calibrateButton = Button(contentFrame, text = "Calibrate robot at start")
    attrStr = 'startWithCalibration'
    buttonObj = calibrateButton
    buttonDict.update({"calibrateButton" : buttonObj})
    calibrateButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
    buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

    # optiButton = Button(contentFrame, text = "Use OptiTrack")
    # attrStr = 'useOptitrack'
    # buttonObj = optiButton
    # buttonDict.update({"optiButton" : buttonObj})
    # optiButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
    # buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')



    omniButton = Button(contentFrame, text = "Use haptic device")
    attrStr = 'useOmni'
    buttonObj = omniButton
    buttonDict.update({"omniButton" : buttonObj})
    omniButton.config(command = partial(toggleInputButton, settingsClass, attrStr, buttonObj))
    buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')


    # pathButton = Button(rootWindow, text = "Follow preprogrammed path")
    # attrStr = 'usePathFile'
    # buttonObj = optiButton
    # buttonDict.update({"optiButton" : buttonObj})
    # pathButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
    # buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')


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
    stopButton.config(command = partial(stopFunction, settingsClass, buttonObj, moveButton, pandaViewerProcess))

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

    grasperLabel = Label(contentFrame, text = "Grasper")
    labelObj = grasperLabel
    labelDict.update({"grasperLabel" : labelObj})



    
    # Creates slider to rotate input device coordinates.
    # To be placed below the pressure bars, so it is the same width as the pressure bar canvas
    rotationSlider = Scale(contentFrame, from_=180, to=-180, length = 400, tickinterval=60, orient = HORIZONTAL)
    labelDict.update({"rotationSlider" : rotationSlider})

    zeroAngle = 0
    zeroPress = 0
    buttonValue = 0
    pumpController = threadArdComms.ardThreader()
    # startThreader opens the serial connection and starts the communication thread
    pumpController.startThreader()
    pumpsConnected = pumpController.connected
    HOLD_MODE = 1
    ISOLATE_P_SUPPLY = 1

    if pumpsConnected:
        n = 1
        for x in range(n):
            pumpController.sendStep(zeroAngle, zeroAngle, zeroAngle, zeroAngle, zeroPress, HOLD_MODE, ISOLATE_P_SUPPLY, buttonValue)
            time.sleep(0.2)
            print(x)

    print("Connected to Control Unit? ", pumpsConnected)
    labelDict["pumpLabel"].config(fg = "green") if pumpsConnected else labelDict["pumpLabel"].config(fg = "red")

    # Set command for move Robot button, taking in label dictionary
    moveButton.config(command = lambda : threading.Thread(target = moveRobot, args = [buttonDict, labelDict, settingsClass, pumpController, viewerInput]).start())



    #################################################################
    # Placement

    contentFrame.grid(column=0, row=0)
    yPadding = 10
    xPadding = 10


    headingSLabel.grid(column = 0, row = 0, pady = yPadding, padx = xPadding)
    headingLLabel.grid(column = 1, row = 0, pady = yPadding, padx = xPadding)
    headingPLabel.grid(column = 4, row = 0, columnspan = 3, pady = yPadding, padx = xPadding)
    # pressCanvas.grid(column = 3, row = 2, rowspan = 6, columnspan = 4, pady = yPadding, padx = xPadding)

    # Place buttons
    rowZerothColumn = 1
    columnNo = 0
    for b in buttonDict:
        if b not in ["moveButton", "stopButton", "resetButton"]:
            buttonDict[b].grid(column = columnNo, row = rowZerothColumn, pady = yPadding, padx = xPadding)
            rowZerothColumn = rowZerothColumn + 1
        else:
            buttonDict[b].grid(column = columnNo, row = rowZerothColumn + 2, pady = yPadding, padx = xPadding)
            columnNo = columnNo + 1
    # columnNo = 0
    # homeButton.grid(column = columnNo, row = rowZerothColumn + 2, pady = yPadding, padx = xPadding)
    moveButtonRow = rowZerothColumn

    # columnNo = 1
    # rotationSlider.grid(column = columnNo, row = rowZerothColumn + 1, columnspan = 4, pady = yPadding, padx = xPadding)

    # Place labels
    rowFirstColumn = 1
    columnNo = 1
    for l in labelDict:
        if l not in ["rotationSlider"]: # Exclude rotation slider here
            labelDict[l].grid(column = columnNo, row = rowFirstColumn, pady = yPadding, padx = xPadding)
            rowFirstColumn = rowFirstColumn + 1


      # Offset buttons
    # Dynamically create 6 joint controls
    jointLabelList = ["Axial", "Rotary", "Tool Ext", "Wrist", "Grasper", "Extra"]
    joints = []
    for i in range(1, 7):
        joint_row = jointOffsets.JointControl(contentFrame, joint_id=i, rowNo=i, colNo=3)
        joint_row.lbl_name.config(text = jointLabelList[i-1])
        joints.append(joint_row)

    settingsClass.jointOffsetButtons = joints



    # Set what to do when window is closed
    rootWindow.protocol("WM_DELETE_WINDOW", partial(onClosing, settingsClass, buttonDict, pandaViewerProcess))

    # This is where the magic happens
    sv_ttk.set_theme("dark")

    #Begin Tk loop

    while (settingsClass.destroyWindow is False):
        rootWindow.update_idletasks()
        rootWindow.update()
        if pumpsConnected:
            if (settingsClass.moveRobotRunning == False):
                [realStepL, realStepR, realStepT, realStepP], [pressL, pressR, pressT, pressP, regulatorSensor], timeL, [loadL, loadR, loadT, loadP] = pumpController.getData()


    rootWindow.destroy()
    pumpController.closeSerial()
    #TODO close threads and disconnect properly if window closed
    # make a dict of buttons, not a list
    # add status labels e.g. to show if arduinos connected
    # add bars that change colour/height depending on pressure (normalse to MAX_PRESS)














