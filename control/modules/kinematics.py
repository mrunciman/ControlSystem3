"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt


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
        self.MIN_EXTEND = 0.0 # mm
        self.MAX_EXTEND = 100 # mm
        self.LENGTH_WRIST = 16
        # Set limit when curvature of continuum joint is assumed zero
        self.MIN_CONT_RAD = 0.1 # mm


        ###################################################################
        # Gantry robot geometry and limits
        ###################################################################
        # Limits on angle change of torque coil / rotary axis
        self.MIN_ROTARY = -100.0   # degrees 
        self.MAX_ROTARY =  100.0   # degrees
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
        self.MAX_TOOL_EXT = 60    # mm
        # Geometry of tool extension motor 
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
    


    def inverseGantry(self, x, y, z, phi):

        rho = mt.hypot(x, y)

        if abs(phi) < 1e-8:
            A = self.LENGTH_WRIST
            B = 0.0
        else:
            A = self.LENGTH_WRIST * mt.sin(phi) / phi
            B = self.LENGTH_WRIST * (1.0 - mt.cos(phi)) / phi

        if abs(mt.sin(phi)) < 1e-8:
            raise ValueError(
                "Singular configuration: prism6 cannot be determined "
                "independently from position when sin(phi) is zero."
            )

        theta1 = mt.atan2(-x, y)

        prism6 = (rho - B) / mt.sin(phi)

        prism0 = z - A - prism6 * mt.cos(phi)

        return prism0, theta1, prism6



    def inverse_kinematics_smooth(
        x, y, z, phi, Lw,
        previous_prism0,
        previous_theta1,
        previous_prism6,
        damping=0.02,
        radial_tolerance=1e-6
    ):
        sphi = mt.sin(phi)
        cphi = mt.cos(phi)

        # Numerically stable constant-curvature terms
        if abs(phi) < 1e-4:
            phi2 = phi * phi
            A = Lw * (1.0 - phi2 / 6.0 + phi2 * phi2 / 120.0)
            B = Lw * (phi / 2.0 - phi * phi2 / 24.0)
        else:
            A = Lw * sphi / phi
            B = Lw * (1.0 - cphi) / phi

        rho = mt.hypot(x, y)

        # theta1 becomes unobservable as rho approaches zero.
        if rho > radial_tolerance:
            measured_theta1 = mt.atan2(-x, y)

            # Unwrap measured_theta1 to be closest to previous_theta1.
            angle_error = mt.atan2(
                mt.sin(measured_theta1 - previous_theta1),
                mt.cos(measured_theta1 - previous_theta1)
            )
            measured_theta1 = previous_theta1 + angle_error

            # Smoothly reduce reliance on x-y direction near the singularity.
            theta_weight = rho * rho / (
                rho * rho + radial_tolerance * radial_tolerance
            )

            theta1 = (
                previous_theta1
                + theta_weight * (measured_theta1 - previous_theta1)
            )
        else:
            theta1 = previous_theta1

        # Signed radial position using the selected theta1.
        radial_position = (
            -x * mt.sin(theta1)
            + y * mt.cos(theta1)
        )

        # Damped solution for prism6:
        # Away from phi=0, approaches (radial_position - B) / sin(phi).
        # At phi=0, approaches previous_prism6.
        damping_squared = damping * damping

        prism6 = (
            sphi * (radial_position - B)
            + damping_squared * previous_prism6
        ) / (
            sphi * sphi + damping_squared
        )

        # Choose prism0 to satisfy z exactly.
        prism0 = z - A - prism6 * cphi

        return prism0, theta1, prism6


