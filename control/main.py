

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

    # Try statement of the main loop
    try:
        deactivateButtons(dictButtons)

        jointOffsetButtons = classSettings.jointOffsetButtons
        motorAngleLabels = classSettings.motorAngleLabels
        useOptitrack = classSettings.useOptitrack
        usePathFile = classSettings.usePathFile
        flagStop = classSettings.stopFlag

        
        ############################################################
        # Instantiate classes:

        kineSolve = kinematics.kineSolver()
        ardLogging = pumpLog.ardLogger()
        posLogging = positionInput.posLogger()
        opTrack = optiStream.optiTracker()

        SAMP_FREQ = 100
        CALIBRATION_MODE = 0
        HOLD_MODE = 1
        ACTIVE_MODE = 2
        ISOLATE_P_SUPPLY = 0
        DEFLATION_MODE = 1
        SET_PRESS_MODE = 3

        HOMING_POSITION = [0, 0, 0]


        pumpDataUpdated = False
        firstMoveDelay = 0
        firstMoveDivider = 100


        ############################################################################
        # Initialise variables 

        flagStop = False

        # Use different methods for different paths
        xPath = []
        yPath = []
        zPath = []
        xMap, yMap, zMap = 0, 0, 0

        ps4 = ps4_pyUSB.ps4USB() # Create an object from controller
        dictLabel["omniLabel"][1] = ps4.controller
        if ps4.controller is not None:
            ps4.start() #start and listen to events
            ps4Buttons = 0
            dictLabel["omniLabel"][0].config(fg = "green")
        else:
            dictLabel["omniLabel"][0].config(fg = "red")

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
                if not xPath:
                    raise ValueError("Path file contains no coordinates")
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


        desAxialPos, desRotaryPos, desToolExt, desWristAngle, desGraspPos = 0, 0, 0, 0, 0
        axialPos, rotaryPos, toolExt, wristAngle, graspPos = 0, 0, 0, 0, 0

        desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp = 0, 0, 0, 0, 0

        # Set initial pressure and calibration variables
        timeL = 0
        prevTimeL = 0
        stepList = [0, 0, 0, 0]
        pressList = [0, 0, 0, 0, 0]
        loadList = [0, 0, 0, 0]

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
        
        dictLabel["pumpLabel"][1] = pumpsConnected
        # dictLabel["pumpLabel"].config(fg = "green") if pumpsConnected else dictLabel["pumpLabel"].config(fg = "red")

        if pumpsConnected:
            pumpController.sendStep(initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp, 
                                    HOLD_MODE, ISOLATE_P_SUPPLY, controllerButtons
                                    )



    ###############################################################################################

        if pumpsConnected:
            time.sleep(1.5)
            if not messagebox.askokcancel("Structure deployed?", "Has the structure been deployed?"):
                raise UserCancelled
            pumpController.sendStep(initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp, 
                                    HOLD_MODE, SET_PRESS_MODE, controllerButtons
                                    )

        else:
            print("MOTOR CONTROLLER NOT CONNECTED. RUNNING WITHOUT MOTOR.")


        if not messagebox.askokcancel("Proceed?", "Start the robot? MANUAL CALIBRATION COMPLETE?"):
            raise UserCancelled

        ################################################################
        # Begin main loop
        next_iteration = time.perf_counter()
        while(not flagStop):
            next_iteration += 1/SAMP_FREQ

            # Get offsest values from GUI
            jointOffsetIndex = 0
            for joint in jointOffsetButtons:
                jointOffsetList[jointOffsetIndex] = joint.offset.get()
                jointOffsetIndex = jointOffsetIndex + 1

            dictLabel["omniLabel"][1] = ps4.controller
            # dictLabel["omniLabel"].config(fg = "green") if ps4.controller is not None else dictLabel["omniLabel"].config(fg = "red")
            if ps4.controller is not None:
                ps4Buttons = ps4.getPSButtonData()
                controllerButtons = ps4Buttons

                # This gives desired joint values 
                [desAxialPos, desRotaryPos, desToolExt, desWristAngle, desGraspPos] = ps4.incrementCylCoords(axialPos, rotaryPos, toolExt, wristAngle, graspPos)
                
                # Convert desired joint values into angular positions of each motor
                intermedAxial, desAxialPos, axialAdjustForWrist = kineSolve.setAxialMotor(desAxialPos, desWristAngle)
                intermedRotary, desRotaryPos = kineSolve.setRotaryMotor(desRotaryPos, rotaryPos)
                intermedTool, desToolExt = kineSolve.setToolMotor(desToolExt)
                intermedWrist, desWristAngle = kineSolve.setWristMotor(desWristAngle)
                intermedGrasp, desGraspPos = kineSolve.setGraspMotor(desGraspPos)
                # print("Motor angles: ", desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, "\n")
                dictLabel["axialLabel"][1] = '{0:.2f}'.format(desAxialPos) + ' mm'
                # dictLabel["axialLabel"].config(text = '{0:.2f}'.format(desAxialPos) + ' mm')
                dictLabel["rotaryLabel"][1] = '{0:.2f}'.format(desRotaryPos) + ' deg'
                # dictLabel["rotaryLabel"].config(text = '{0:.2f}'.format(desRotaryPos) + ' deg')
                dictLabel["toolLabel"][1] = '{0:.2f}'.format(desToolExt) + ' mm'
                # dictLabel["toolLabel"].config(text = '{0:.2f}'.format(desToolExt) + ' mm')
                dictLabel["wristLabel"][1] = '{0:.2f}'.format(desWristAngle) + ' deg'
                # dictLabel["wristLabel"].config(text = '{0:.2f}'.format(desWristAngle) + ' deg')
                dictLabel["graspLabel"][1] = '{0:.2f}'.format(desGraspPos) + ' mm'
                # dictLabel["graspLabel"].config(text = '{0:.2f}'.format(desGraspPos) + ' mm')

                desiredThetaAxial = intermedAxial + jointOffsetList[0]
                desiredThetaRotary = intermedRotary + jointOffsetList[1]
                desiredThetaTool = intermedTool + jointOffsetList[2]
                desiredThetaWrist = intermedWrist + jointOffsetList[3]
                desiredThetaGrasp = intermedGrasp + jointOffsetList[4]

                # frameRotAngle = dictLabel["rotationSlider"].get()
                

            if classSettings.goToHome:
                controllerButtons = 0

            dictLabel["graspLabel"][1] = controllerButtons
            # if controllerButtons == 1:
            #     dictLabel["grasperLabel"].config(text = "Grasper open", fg = "green")
            # elif controllerButtons == 2:
            #     dictLabel["grasperLabel"].config(text = "Grasper close", fg = "red")
            # else:
            #     dictLabel["grasperLabel"].config(text = "Grasper", fg = "white")

            dictLabel["joystickXLabel"][1] = ps4.RstickX
            # if ps4.RstickX > 1e-8:
            #     dictLabel["joystickXLabel"].config(text = 'Joystick LR: +' + '{0:.2f}'.format(ps4.RstickX), fg = "green")
            # elif ps4.RstickX < 1e-8:
            #     dictLabel["joystickXLabel"].config(text = 'Joystick LR: ' + '{0:.2f}'.format(ps4.RstickX), fg = "red")
            # else:
            #     dictLabel["joystickXLabel"].config(text = 'Joystick LR: +' + '{0:.2f}'.format(ps4.RstickX), fg = "white")

            dictLabel["joystickYLabel"][1] = ps4.RstickY
            # if ps4.RstickY > 1e-8:
            #     dictLabel["joystickYLabel"].config(text = 'Joystick UD: +' + '{0:.2f}'.format(ps4.RstickY), fg = "green")
            # elif ps4.RstickY < 1e-8:
            #     dictLabel["joystickYLabel"].config(text = 'Joystick UD: ' + '{0:.2f}'.format(ps4.RstickY), fg = "red")
            # else:
            #     dictLabel["joystickYLabel"].config(text = 'Joystick UD: +' + '{0:.2f}'.format(ps4.RstickY), fg = "white")

            
            if axialPos < desAxialPos:
                dictLabel["R1R2Label"][1] = 'R2 - Out'
            elif axialPos > desAxialPos:
                dictLabel["R1R2Label"][1] = 'R1 - In' 
            else:dictLabel["R1R2Label"][1] = 'R1R2'


            
            if desToolExt < toolExt:
                dictLabel["extendLabel"][1] = 1
                # dictLabel["extendLabel"].config(text = "Tool retract", fg = "orange")
            elif desToolExt > toolExt:
                dictLabel["extendLabel"][1] = 2
                # dictLabel["extendLabel"].config(text = "Tool extend", fg = "cyan")
            else:
                dictLabel["extendLabel"][1] = 3
                # dictLabel["extendLabel"].config(text = "Tool", fg = "white")


            visualiseOffset = 0
            viewerInputList[0] = desAxialPos + axialAdjustForWrist + jointOffsetList[0]*visualiseOffset
            viewerInputList[1] = desRotaryPos   + jointOffsetList[1]*visualiseOffset
            viewerInputList[2] = desToolExt     + jointOffsetList[2]*visualiseOffset
            viewerInputList[3] = desWristAngle  + jointOffsetList[3]*visualiseOffset
            viewerInputList[4] = desGraspPos    + jointOffsetList[4]*visualiseOffset
            # print("From tkinter ", viewerInputList)

    
            # desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp
            # Log desired positions
            if pumpDataUpdated:
                robotX = viewerInputList[5]
                robotY = viewerInputList[6]
                robotZ = viewerInputList[7]
                robotRoll = viewerInputList[8]
                robotPitch = viewerInputList[9]
                robotYaw = viewerInputList[10]
                posLogging.posLog(robotX, robotY, robotZ, robotRoll, robotPitch, robotYaw)
                #Show position on GUI
                dictLabel["robotXLabel"][1] = '{0:.2f}'.format(robotX) + ' mm'
                # dictLabel["robotXLabel"].config(text = '{0:.2f}'.format(robotX) + ' mm')
                dictLabel["robotYLabel"][1] = '{0:.2f}'.format(robotY) + ' mm'
                # dictLabel["robotYLabel"].config(text = '{0:.2f}'.format(robotY) + ' mm')
                dictLabel["robotZLabel"][1] = '{0:.2f}'.format(robotZ) + ' mm'
                # dictLabel["robotZLabel"].config(text = '{0:.2f}'.format(robotZ) + ' mm')
                dictLabel["robotRollLabel"][1] = '{0:.2f}'.format(robotRoll) + ' deg'
                # dictLabel["robotRollLabel"].config(text = '{0:.2f}'.format(robotRoll) + ' deg')
                dictLabel["robotPitchLabel"][1] = '{0:.2f}'.format(robotPitch) + ' deg'
                # dictLabel["robotPitchLabel"].config(text = '{0:.2f}'.format(robotPitch) + ' deg')
                dictLabel["robotYawLabel"][1] = '{0:.2f}'.format(robotYaw) + ' deg'
                # dictLabel["robotYawLabel"].config(text = '{0:.2f}'.format(robotYaw) + ' deg')

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
                    pumpController.sendStep(
                        initThetaAxial, initThetaRot, initThetaTool, initThetaWrist, initThetaGrasp,
                         ACTIVE_MODE, SET_PRESS_MODE, controllerButtons
                         )
                else:
                    # Send step number to arduinos:
                    pumpController.sendStep(
                        desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp,
                         ACTIVE_MODE, SET_PRESS_MODE, controllerButtons
                         )

                # Log values from arduinos
                if pumpDataUpdated:
                    ardLogging.ardLogAll(stepList, pressList, timeL, loadList)

                # Get current pump position, pressure and times from arduinos
                stepList, pressList, timeL, loadList = pumpController.getData()
                # [realStepL, realStepR, realStepT, realStepP] = stepList
                # [pressL, pressR, pressT, pressP, regulatorSensor] = pressList
                # [loadL, loadR, loadT, loadP] = loadList

            # Check if new data has been received from pump controller 
            if (timeL - prevTimeL > 0):
                pumpDataUpdated = True
                kineSolve.TIMESTEP = (timeL - prevTimeL)/1000
                # kineSolve.TIMESTEP = 0.01
                if (kineSolve.TIMESTEP < 0.01):
                    kineSolve.TIMESTEP = 1/SAMP_FREQ
                # print((timeL - prevTimeL)/1000)
            else:
                pumpDataUpdated = False
                kineSolve.TIMESTEP = 1/SAMP_FREQ

            # Update current position, cable lengths, and volumes as previous targets
            prevTimeL = timeL
            prevPathCounter = pathCounter
            pathCounter += 1
            # if pumpDataUpdated: pathCounter += 1
            axialPos, rotaryPos, toolExt, wristAngle, graspPos = desAxialPos, desRotaryPos, desToolExt, desWristAngle, desGraspPos

            # Stop operation if Stop button hit
            flagStop = classSettings.stopFlag

            # remaining = next_iteration - time.perf_counter()
            # # print(remaining)
            # if remaining > 0:
            #     time.sleep(remaining)
            # else:
            #     next_iteration = time.perf_counter()

    except UserCancelled:
        print("Operation cancelled by user.")
    
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
        print("Ending loop...")

        ###########################################################################
        # Stop program
        # Disable pumps and set them to idle state
        try:

            if pumpsConnected:
                # Save values gathered from arduinos
                ardLogging.ardLogAll(stepList, pressList, timeL, loadList)
                # Save joint information
                ardLogging.ardSave()
                # Ensure same number of rows in position log file
                robotX = viewerInputList[5]
                robotY = viewerInputList[6]
                robotZ = viewerInputList[7]
                robotRoll = viewerInputList[8]
                robotPitch = viewerInputList[9]
                robotYaw = viewerInputList[10]
                posLogging.posLog(robotX, robotY, robotZ, robotRoll, robotPitch, robotYaw)
                #Save position data
                posLogging.posSave()

                # if calibrated:
                controllerButtons = 0
                pumpController.sendStep(
                    desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, 
                    HOLD_MODE, DEFLATION_MODE, controllerButtons
                    )
                time.sleep(0.2)
                n = 20
                for x in range(n):
                    pumpController.sendStep(desiredThetaAxial, desiredThetaRotary, desiredThetaTool, desiredThetaWrist, desiredThetaGrasp, 
                                            HOLD_MODE, DEFLATION_MODE, controllerButtons
                                            )
                    time.sleep(0.2)
                    # print(x)
                time.sleep(0.2)
                stepList, pressList, timeL, loadList = pumpController.getData()
                # [realStepL, realStepR, realStepT, realStepP] = stepList
                # [pressL, pressR, pressT, pressP, regulatorSensor] = pressList
                # [loadL, loadR, loadT, loadP] = loadList


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
        # Reactivate selection buttons 
        activateButtons(dictButtons, flagStop)





###################################################################################################
###################################################################################################


class controlSettings:
    def __init__(self):
        self.jointOffsetButtons = None
        self.pandaViewerProcess = None
        self.robotThread = None
        self.startWithCalibration = False
        self.useOmni = False
        self.socketOmni = None
        self.useOptitrack = False
        self.motorAngleLabels = None
        self.moveRobotRunning = False
        self.usePathFile = False
        self.goToHome = False
        self.stopFlag = False
        self.socketFalcon = None
        self.destroyWindow = False

class UserCancelled(Exception):
    pass

def startFunction(
    classSettings,
    dictButtons,
    dictLabels,
    pumpController,
    viewerInput 
    ):  

    # Prevent two control loops from running simultaneously
    if classSettings.moveRobotRunning:
        print("Robot is already running.")
        return

    # A multiprocessing.Process cannot be restarted, so create a new one
    viewerProcess = multiprocessing.Process(
        target=viewer_process,
        args=(viewerInput,)
    )
    viewerProcess.daemon = True

    classSettings.pandaViewerProcess = viewerProcess
    classSettings.stopFlag = False
    classSettings.moveRobotRunning = True

    viewerProcess.start()

    robotThread = threading.Thread(
        target=moveRobot,
        args=(
            dictButtons,
            dictLabels,
            classSettings,
            pumpController,
            viewerInput
        ),
        name="robotControlThread",
        daemon=True
    )

    classSettings.robotThread = robotThread
    robotThread.start()


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


def stopFunction(classSettings, stopButton, startButton):
    if not classSettings.moveRobotRunning:
        print("Robot is not currently running.")
        return

    # Signal the robot-control thread to finish
    classSettings.stopFlag = True
    stopButton.config(bg="red")
    startButton.config(state="disabled")

    # Stop the viewer that belongs to the current experiment
    viewerProcess = classSettings.pandaViewerProcess

    if viewerProcess is not None:
        if viewerProcess.is_alive():
            viewerProcess.terminate()
            viewerProcess.join(timeout=2.0)

        if viewerProcess.is_alive():
            print("Viewer process did not terminate cleanly; killing it.")
            viewerProcess.kill()
            viewerProcess.join(timeout=2.0)

    classSettings.pandaViewerProcess = None


def resetFunction(classSettings, stopButton, startButton):
    robotThread = classSettings.robotThread
    if (classSettings.moveRobotRunning or 
        (robotThread is not None and robotThread.is_alive())):
            print("Cannot reset while the robot is still stopping.")
            return

    classSettings.robotThread = None
    classSettings.pandaViewerProcess = None
    classSettings.stopFlag = False
    stopButton.config(bg = '#1c1c1c')
    startButton.config(state = 'normal')


def onClosing(classSettings, dictButtons, pumpController):
    # Exit control loop properly
    if not messagebox.askokcancel("Quit", "Do you want to quit?"):
        return
    classSettings.stopFlag = True
    classSettings.destroyWindow = True
    dictButtons['stopButton'].config(bg = 'red')
    dictButtons['moveButton'].config(state = 'disabled')
    viewerProcess = classSettings.pandaViewerProcess
    if viewerProcess is not None and viewerProcess.is_alive():
        viewerProcess.terminate()
        viewerProcess.join(timeout = 10.0)

        if viewerProcess.is_alive():
            viewerProcess.kill()
            viewerProcess.join(timeout = 2.0)

    classSettings.pandaViewerProcess = None            

    for thread in threading.enumerate():
        if thread.name != "MainThread":
            if thread.name == "ardThread":
                pumpController.stopThreader()
                pumpController.t.stop()
                pumpController.closeSerial()
            else:
                thread._connection_made.set()
                thread.join(timeout = 2.0)
                if thread.is_alive():
                    print(f"Thread did not stop: {thread.name}")        


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


def viewer_process(angleList):
    pandaViewer.run_viewer(angleList)

########################################################################################################
# GUI
########################################################################################################
if __name__ == '__main__':
    rootWindow = Tk()
    rootWindow.title("Lannsair Endoluminal Robotics")
    rootWindow.geometry("900x450")

    contentFrame = ttk.Frame(rootWindow)
    winStyle = ttk.Style()
    winStyle.theme_use("classic")

    settingsClass = controlSettings()

    manager = multiprocessing.Manager()

    initThetaAxial, initThetaRotary, initThetaTool, initThetaWrist, initThetaGrasp, localX, localY, localZ, localRoll, localPitch, localYaw = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    viewerInput = manager.list([initThetaAxial, initThetaRotary, initThetaTool, initThetaWrist, initThetaGrasp, localX, localY, localZ, localRoll, localPitch, localYaw])

    # Headings
    headingSLabel = Label(contentFrame, text = "Settings", font='bold')
    headingLLabel = Label(contentFrame, text = "Status", font='bold')
    headingPLabel = Label(contentFrame, text = "Offsets", font='bold')
    headingJointLabel = Label(contentFrame, text = "Joint Values", font='bold')
    headingPosLabel = Label(contentFrame, text = "Pose", font='bold')

    # Create buttons

    buttonDict = {}

    # calibrateButton = Button(contentFrame, text = "Calibrate robot at start")
    # attrStr = 'startWithCalibration'
    # buttonObj = calibrateButton
    # buttonDict.update({"calibrateButton" : buttonObj})
    # calibrateButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
    # buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

    # optiButton = Button(contentFrame, text = "Use OptiTrack")
    # attrStr = 'useOptitrack'
    # buttonObj = optiButton
    # buttonDict.update({"optiButton" : buttonObj})
    # optiButton.config(command = partial(toggleButton, settingsClass, attrStr, buttonObj))
    # buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

    # omniButton = Button(contentFrame, text = "Use haptic device")
    # attrStr = 'useOmni'
    # buttonObj = omniButton
    # buttonDict.update({"omniButton" : buttonObj})
    # omniButton.config(command = partial(toggleInputButton, settingsClass, attrStr, buttonObj))
    # buttonObj.config(bg = 'green') if vars(settingsClass)[attrStr] else buttonObj.config(bg = 'red')

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

    resetButton = Button(contentFrame, text = "Reset")
    buttonObj = resetButton
    buttonDict.update({"resetButton" : buttonObj})


    # Status labels
    labelDict = {}

    pumpLabel = Label(contentFrame, text = "Pump controller connection")
    pumpValue = 0
    pumpList = [pumpLabel, pumpValue]
    labelDict.update({"pumpLabel" : pumpList})
    # print(labelDict["pumpLabel"][0])

    omniLabel = Label(contentFrame, text = "Controller connected")
    omniValue = 0
    omniList = [omniLabel, omniValue]
    labelDict.update({"omniLabel" : omniList})

    # calibrationLabel = Label(contentFrame, text = "Calibration")
    # labelObj = calibrationLabel
    # labelDict.update({"calibrationLabel" : labelObj})
    R1R2Label = Label(contentFrame, text = "R1R2")
    R1R2Value = 0
    R1R2List = [R1R2Label, R1R2Value]
    labelDict.update({"R1R2Label" : R1R2List})

    joystickXLabel = Label(contentFrame, text = "Joystick LR: ")
    joystickXValue = 0
    joystickXList = [joystickXLabel, joystickXValue]
    labelDict.update({"joystickXLabel" : joystickXList})

    extendLabel = Label(contentFrame, text = "Tool")
    extendValue = 0
    extendList = [extendLabel, extendValue]
    labelDict.update({"extendLabel" : extendList})

    joystickYLabel = Label(contentFrame, text = "Joystick UD: ")
    joystickYValue = 0
    joystickYList = [joystickYLabel, joystickYValue]
    labelDict.update({"joystickYLabel" : joystickYList})

    grasperLabel = Label(contentFrame, text = "Grasper")
    grasperValue = 0
    grasperList = [grasperLabel, grasperValue]
    labelDict.update({"grasperLabel" : grasperList})

    axialValueLabel = Label(contentFrame, text = "0.00 mm")
    axialValueValue = 0
    axialValueList = [axialValueLabel, axialValueValue]
    labelDict.update({"axialLabel" : axialValueList})

    rotaryValueLabel = Label(contentFrame, text = "0.00 deg")
    rotaryValueValue = 0
    rotaryValueList = [rotaryValueLabel, rotaryValueValue]
    labelDict.update({"rotaryLabel" : rotaryValueList})

    toolValueLabel = Label(contentFrame, text = "0.00 mm")
    toolValueValue = 0
    toolValueList = [toolValueLabel, toolValueValue]
    labelDict.update({"toolLabel" : toolValueList})

    wristValueLabel = Label(contentFrame, text = "0.00 deg")
    wristValueValue = 0
    wristValueList = [wristValueLabel, wristValueValue]
    labelDict.update({"wristLabel" : wristValueList})

    graspValueLabel = Label(contentFrame, text = "0.00 mm")
    graspValueValue = 0
    graspValueList = [graspValueLabel, graspValueValue]
    labelDict.update({"graspLabel" : graspValueList})

    posXLabel = Label(contentFrame, text = "0.00 mm")
    posXValue = 0
    posXList = [posXLabel, posXValue]
    labelDict.update({"robotXLabel" : posXList})

    posYLabel = Label(contentFrame, text = "0.00 mm")
    posYValue = 0
    posYList = [posYLabel, posYValue]
    labelDict.update({"robotYLabel" : posYList})

    posZLabel = Label(contentFrame, text = "0.00 mm")
    posZValue = 0
    posZList = [posZLabel, posZValue]
    labelDict.update({"robotZLabel" : posZList})

    rollLabel = Label(contentFrame, text = "0.00 deg")
    rollValue = 0
    rollList = [rollLabel, rollValue]
    labelDict.update({"robotRollLabel" : rollList})

    pitchLabel = Label(contentFrame, text = "0.00 deg")
    pitchValue = 0
    pitchList = [pitchLabel, pitchValue]
    labelDict.update({"robotPitchLabel" : pitchList})

    yawLabel = Label(contentFrame, text = "0.00 deg")
    yawValue = 0
    yawList = [yawLabel, yawValue]
    labelDict.update({"robotYawLabel" : yawList})

    # Creates slider to rotate input device coordinates.
    # To be placed below the pressure bars, so it is the same width as the pressure bar canvas
    rotationSlider = Scale(contentFrame, from_=180, to=-180, length = 400, tickinterval=60, orient = HORIZONTAL)
    # labelDict.update({"rotationSlider" : rotationSlider})

    buttonValue = 0
    pumpController = threadArdComms.ardThreader()
    # startThreader opens the serial connection and starts the communication thread
    pumpController.startThreader()
    pumpsConnected = pumpController.connected
    HOLD_MODE = 1
    ISOLATE_P_SUPPLY = 1

    if pumpsConnected:
        time.sleep(3)
        n = 1
        for x in range(n):
            pumpController.sendStep(initThetaAxial, initThetaRotary, initThetaTool, initThetaWrist, initThetaGrasp, 
                                    HOLD_MODE, ISOLATE_P_SUPPLY, buttonValue
                                    )
            time.sleep(0.2)

    print("Connected to Control Unit? ", pumpsConnected)
    labelDict["pumpLabel"][1] = pumpsConnected
    labelDict["pumpLabel"][0].config(fg = "green") if pumpsConnected else labelDict["pumpLabel"][0].config(fg = "red")

    # Set command for move Robot button, taking in label dictionary
    # moveButton.config(command = lambda : threading.Thread(target = moveRobot, args = [buttonDict, labelDict, settingsClass, pumpController, viewerInput]).start())
    moveButton.config(
        command=partial(
            startFunction,
            settingsClass,
            buttonDict,
            labelDict,
            pumpController,
            viewerInput
        )
    )

    stopButton.config(
        command=partial(
            stopFunction,
            settingsClass,
            stopButton,
            moveButton
        )
    )

    resetButton.config(
        command=partial(
            resetFunction,
            settingsClass,
            stopButton,
            moveButton
        )
    )


    #################################################################
    # Placement

    contentFrame.grid(column=0, row=0)
    yPadding = 10
    xPadding = 10


    headingSLabel.grid(column = 0, row = 0, pady = yPadding, padx = xPadding)
    headingLLabel.grid(column = 1, row = 0, pady = yPadding, padx = xPadding)
    headingPLabel.grid(column = 4, row = 0, columnspan = 3, pady = yPadding, padx = xPadding)
    headingJointLabel.grid(column = 7, row = 0, pady = yPadding, padx = xPadding)
    headingPosLabel.grid(column = 9, row = 0, pady = yPadding, padx = xPadding)
    # pressCanvas.grid(column = 3, row = 2, rowspan = 6, columnspan = 4, pady = yPadding, padx = xPadding)

    # Place buttons
    rowZerothColumn = 1
    columnNo = 0
    for b in buttonDict:
        if b not in ["moveButton", "stopButton", "resetButton"]:
            buttonDict[b].grid(column = columnNo, row = rowZerothColumn, pady = yPadding, padx = xPadding)
            rowZerothColumn = rowZerothColumn + 1
        else:
            buttonDict[b].grid(column = columnNo, row = rowZerothColumn + 6, pady = yPadding, padx = xPadding)
            columnNo = columnNo + 1
    # columnNo = 0
    # homeButton.grid(column = columnNo, row = rowZerothColumn + 2, pady = yPadding, padx = xPadding)
    moveButtonRow = rowZerothColumn

    # columnNo = 1
    # rotationSlider.grid(column = columnNo, row = rowZerothColumn + 1, columnspan = 4, pady = yPadding, padx = xPadding)

    # Place labels
    rowFirstColumn = 2
    columnNo = 0
    for l in labelDict:
        if l in ["R1R2Label"]:
            columnNo = 1
            rowFirstColumn =1
        if l in ["axialLabel"]:
            columnNo = 7
            rowFirstColumn = 1
        elif l in ["robotXLabel"]:
            columnNo = 9
            rowFirstColumn = 1

        if l not in ["rotationSlider"]: # Exclude rotation slider here
            labelDict[l][0].grid(column = columnNo, row = rowFirstColumn, pady = yPadding, padx = xPadding)
            rowFirstColumn = rowFirstColumn + 1


      # Offset buttons
    # Dynamically create 6 joint controls
    jointLabelList = ["Axial", "Rotary", "Tool Ext", "Wrist", "Grasper"]#, "Extra"]
    joints = []
    for i in range(1, len(jointLabelList)+1):
        joint_row = jointOffsets.JointControl(contentFrame, joint_id=i, rowNo=i, colNo=3)
        joint_row.lbl_name.config(text = jointLabelList[i-1])
        joints.append(joint_row)

    settingsClass.jointOffsetButtons = joints


    # Set what to do when window is closed
    rootWindow.protocol("WM_DELETE_WINDOW", partial(onClosing, settingsClass, buttonDict, pumpController))

    # This is where the magic happens
    sv_ttk.set_theme("dark")

    #Begin Tk loop
    updateGUI_limit = 1000
    updateGUI_count = 0

    while (settingsClass.destroyWindow is False):
        rootWindow.update_idletasks()
        rootWindow.update()

        if (settingsClass.moveRobotRunning):
            updateGUI_count = updateGUI_count + 1
            if updateGUI_count == updateGUI_limit:
                updateGUI_count = 0
                labelDict["omniLabel"][0].config(fg = "green") if labelDict["omniLabel"][1] is not None else labelDict["omniLabel"][0].config(fg = "red")
                labelDict["pumpLabel"][0].config(fg = "green") if labelDict["pumpLabel"][1] else labelDict["pumpLabel"][0].config(fg = "red")
                labelDict["axialLabel"][0].config(text  = labelDict["axialLabel"][1])
                labelDict["rotaryLabel"][0].config(text = labelDict["rotaryLabel"][1])
                labelDict["toolLabel"][0].config(text   = labelDict["toolLabel"][1])
                labelDict["wristLabel"][0].config(text  = labelDict["wristLabel"][1])
                labelDict["graspLabel"][0].config(text  = labelDict["graspLabel"][1])


                labelDict["R1R2Label"][0].config(text = labelDict["R1R2Label"][1])


                if labelDict["graspLabel"][1] == 1:
                    labelDict["grasperLabel"][0].config(text = "Grasper open", fg = "green")
                elif labelDict["graspLabel"][1] == 2:
                    labelDict["grasperLabel"][0].config(text = "Grasper close", fg = "red")
                else:
                    labelDict["grasperLabel"][0].config(text = "Grasper", fg = "white")


                joyXString = 'Joystick LR: +' + '{0:.2f}'.format(labelDict["joystickXLabel"][1])
                if labelDict["joystickXLabel"][1] > 1e-8:
                    labelDict["joystickXLabel"][0].config(text = joyXString, fg = "green")
                elif labelDict["joystickXLabel"][1] < 1e-8:
                    labelDict["joystickXLabel"][0].config(text = joyXString, fg = "red")
                else:
                    labelDict["joystickXLabel"][0].config(text = joyXString, fg = "white")


                joyYString = 'Joystick UD: +' + '{0:.2f}'.format(labelDict["joystickYLabel"][1])
                if labelDict["joystickYLabel"][1] > 1e-8:
                    labelDict["joystickYLabel"][0].config(text = joyYString, fg = "green")
                elif labelDict["joystickYLabel"][1] < 1e-8:
                    labelDict["joystickYLabel"][0].config(text = joyYString, fg = "red")
                else:
                    labelDict["joystickYLabel"][0].config(text = joyYString, fg = "white")
                

                if labelDict["extendLabel"][1] == 1:
                    labelDict["extendLabel"][0].config(text = "Tool retract",fg = "red")
                elif labelDict["extendLabel"][1] == 2:
                    labelDict["extendLabel"][0].config(text = "Tool extend", fg = "green")
                else:
                    labelDict["extendLabel"][0].config(text = "Tool", fg = "white")


                labelDict["robotXLabel"][0].config(text     = labelDict["robotXLabel"][1])
                labelDict["robotYLabel"][0].config(text     = labelDict["robotYLabel"][1])
                labelDict["robotZLabel"][0].config(text     = labelDict["robotZLabel"][1])
                labelDict["robotRollLabel"][0].config(text  = labelDict["robotRollLabel"][1])
                labelDict["robotPitchLabel"][0].config(text = labelDict["robotPitchLabel"][1])
                labelDict["robotYawLabel"][0].config(text   = labelDict["robotYawLabel"][1])



        # if pumpsConnected:
        #     if (settingsClass.moveRobotRunning):
        #         stepList, pressList, timeL, loadList = pumpController.getData()
                # print(stepList)


    rootWindow.destroy()
    pumpController.closeSerial()
