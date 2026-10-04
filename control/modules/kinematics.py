"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

# SIDE_LENGTH = 18.911

class kineSolver:
    # SIDE_LENGTH0 = SIDE_LENGTH
    def __init__(self):
        
        # For step count:
        # Mapping from step to 1 revolution = 200 steps
        # 32 microsteps per step, so 6400 steps per revolution
        # Set M0, M1, M2 to set microstep size
        # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
        # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
        #          = 200*4/8 = 100 steps/mm
        # For speed: pulses/mm * mm/s = pulses/s
        self.STEPS_PER_REV = 200
        self.MICROSTEPS = 16
        self.MICROSTEPS_PRI = 4
        self.LEAD = 8
        self.STEPS_PER_MM = (self.STEPS_PER_REV*self.MICROSTEPS)/(self.LEAD) # steps per mm
        self.STEPS_PER_MM_PRI = (self.STEPS_PER_REV*self.MICROSTEPS_PRI)/(self.LEAD) # steps per mm
        self.TIMESTEP = 0.01 # Inverse of sampling frequency on arduinos



        
        ###################################################################
        # Extension/Retraction Geometry
        ###################################################################
        # Set limits on shaft extension
        # self.minShaftExt = self.SHAFT_LENGTH + 1
        self.MIN_EXTEND = 0.5 # mm
        self.MAX_EXTEND = 100 # mm
        self.LENGTH_WRIST = 16
        self.initialPrismLength = 0
        # Set limit when curvature of continuum joint is assumed zero
        self.MIN_CONT_RAD = 0.1 # mm
        # Max angle that hydraulic motors can have is:
        # print(self.volToAngle(self.MAX_VOL))




        ###################################################################
        # Gantry robot geometry and limits
        ###################################################################
        # Limits on angle change of torque coil / rotary axis
        self.MIN_ROTARY = -400.0   # degrees 
        self.MAX_ROTARY =  400.0   # degrees
        self.targDirRot = 1
        self.prevDirRot = 1
        self.antiHystDegRot = 360/360

        # Limits on wrist angle
        self.MIN_WRIST_ANGLE = 0   # degrees 
        self.MAX_WRIST_ANGLE = 120  # degrees 
        # Geometry of wrist motor
        self.HYPOT_TIP = 5.62      # mm
        self.TIP_CUTAWAY_WIDTH = 2.34 # mm
        self.NUM_SUBSECTIONS = 6   # number of cutaways
        self.RADIUS_SPOOL = 5      # mm
        self.THETA_REST = 2*mt.asin((self.TIP_CUTAWAY_WIDTH/2)/self.HYPOT_TIP)
        self.WRIST_THETA_0 = mt.radians(24)
        self.WRIST_D_0 = 2*self.HYPOT_TIP*mt.sin(self.WRIST_THETA_0/2)

        # Limits on how far instrument can be extended
        self.MIN_TOOL_EXT = -1000   # mm
        self.MAX_TOOL_EXT = 1000    # mm
        # Geometry of tool extensionm motor 
        self.TOOL_EXT_ROLLER_RADIUS = 7.5/2 #mm

        # Limits on grasper control
        self.MIN_GRASP_POS = 0   # mm 
        self.MAX_GRASP_POS = 2.5   # mm




    def setAxialMotor(self, desAxialPos, desWristAngle):
        phi = mt.radians(desWristAngle)
        if abs(phi) < 1e-8:
            axialAdjustForWrist = 0
        else:
            axialAdjustForWrist = self.LENGTH_WRIST*(1 - mt.sin(phi)/phi)
        
        # Impose contraction range
        if (desAxialPos < self.MIN_EXTEND):
            desAxialPos = self.MIN_EXTEND
        elif (desAxialPos > self.MAX_EXTEND):
            desAxialPos = self.MAX_EXTEND

        desAxialPosAdjusted = desAxialPos + axialAdjustForWrist

        axialMotorAngle = 360.0*desAxialPosAdjusted/self.LEAD
        axialMotorAngle = round(axialMotorAngle,2)
            
        return axialMotorAngle, desAxialPos


    def setRotaryMotor(self, desRotaryPos, prevRotPos):

        # For anti-hysteresis in prismatic joint, check target direction and current direction of motion:
        if desRotaryPos > prevRotPos:
            self.targDirRot = 1
        elif desRotaryPos < prevRotPos:
            self.targDirRot = -1
        # If stopped, preserve previous direction as target direction: 
        elif desRotaryPos == prevRotPos:
            self.targDirRot = self.prevDirRot

        self.prevDirRot = self.targDirRot

        # TODO define MIN and MAX ROTARY angles
        if (desRotaryPos < self.MIN_ROTARY):
            desRotaryPos = self.MIN_ROTARY
        elif (desRotaryPos > self.MAX_ROTARY):
            desRotaryPos = self.MAX_ROTARY

        rotaryMotorAngle = desRotaryPos + self.targDirRot*self.antiHystDegRot
        rotaryMotorAngle = round(rotaryMotorAngle,2)

        return rotaryMotorAngle, desRotaryPos


    def setToolMotor(self, desToolExt):

        if (desToolExt < self.MIN_TOOL_EXT):
            desToolExt = self.MIN_TOOL_EXT
        elif (desToolExt > self.MAX_TOOL_EXT):
            desToolExt = self.MAX_TOOL_EXT
            
        toolMotorAngle = mt.degrees(desToolExt/self.TOOL_EXT_ROLLER_RADIUS)
        toolMotorAngle = round(toolMotorAngle,2)

        return toolMotorAngle, desToolExt


    def setWristMotor(self, desWristAngle):
        if (desWristAngle < self.MIN_WRIST_ANGLE):
            desWristAngle = self.MIN_WRIST_ANGLE
        elif (desWristAngle > self.MAX_WRIST_ANGLE):
            desWristAngle = self.MAX_WRIST_ANGLE

        # wrist_d_new = 2*self.HYPOT_TIP*mt.sin((self.WRIST_THETA_0 - mt.radians(desWristAngle)/self.NUM_SUBSECTIONS)/2)
        # wrist_total_d = self.NUM_SUBSECTIONS*(self.WRIST_D_0 - wrist_d_new)
        # wristMotorAngle = wrist_total_d/self.RADIUS_SPOOL

        wristMotorAngle = round(desWristAngle,2)

        # wristMotorAngle = (2*self.HYPOT_TIP/self.RADIUS_SPOOL)  \
        #       *mt.sin((self.THETA_REST - (desWristAngle/self.NUM_SUBSECTIONS))/2)
        
        return wristMotorAngle, desWristAngle



    def setGraspMotor(self, desGraspPos):
        #TODO Set limits on grasper motor angles

        if (desGraspPos < self.MIN_GRASP_POS):
            desGraspPos = self.MIN_GRASP_POS
        elif (desGraspPos > self.MAX_GRASP_POS):
            desGraspPos = self.MAX_GRASP_POS

    
        graspMotorAngle = 360.0*desGraspPos/self.LEAD
        graspMotorAngle = round(graspMotorAngle,2)
        return graspMotorAngle, desGraspPos


