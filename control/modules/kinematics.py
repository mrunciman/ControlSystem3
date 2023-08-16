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
    def __init__(self, triangleSide):

        ##############################################################
        # Structure and actuators
        ##############################################################
        # Define parameters of system:
        # Side length of equilateral triangle in mm
        self.SIDE_LENGTH = triangleSide
        self.OFFSET_X = self.SIDE_LENGTH/2
        self.OFFSET_Y = self.SIDE_LENGTH/2 * mt.tan(mt.pi/6)

        # 'Flat' muscle length:
        self.L_0 = 30

        # Excess length of cable between entry point and muscle, in mm
        # self.Lx = 10
        # Total length of cable in flat/resting state
        # self.Lt = self.L_0 + self.Lx + self.SIDE_LENGTH
        # print(Lt)
        # Equivalent width of hydraulic muscle in mm
        # self.ACT_WIDTH = 18
        self.D_s = 12 # Flat section of actutor
        self.D_t = 30 # Total width of actuator
        self.D_c = (self.D_t - self.D_s)/2 # Width of each individual conic end
        # Number of length subdivisions
        self.NUM_L = 3
        self.FACT_V = ((self.L_0**2)/self.NUM_L)*(self.D_c/3 + self.D_s/2)
        # Syringe cross sectional area, diameter = 26.5 mm
        self.SYRINGE_RADIUS = 12.5/2
        self.A_SYRINGE = mt.pi*(self.SYRINGE_RADIUS**2) # mm^2
        # Real volume calc: there are numLs beams of length L0/numLs
        # self.FACT_V = ((self.ACT_WIDTH/1000)*(self.L_0/1000)**2)/(2*self.NUM_L)
        self.M3_to_MM3 = 1e9
        self.VOL_FACTOR = 1.1 #1.15 #1.09 # 0.9024 # 12.6195/15.066 # Ratio of real volume to theoretical volume
        self.CAL_FACTOR = 0.005 # % of max volume still in actuator after calibration
        self.FACT_ANG = 1
        self.MAX_VOL = self.FACT_V*((mt.pi/2*self.FACT_ANG) - \
            mt.cos((mt.pi/2*self.FACT_ANG))*mt.sin((mt.pi/2*self.FACT_ANG)))/((mt.pi/2*self.FACT_ANG)**2)
        # print(self.MAX_VOL)
        self.DEAD_VOL = self.CAL_FACTOR*self.MAX_VOL
        # print(self.MAX_VOL)
        self.MAX_VOL_RATE = 1000 # mm^3/s
        
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
        self.STEPS_PER_MMCUBED = self.STEPS_PER_MM/self.A_SYRINGE # Steps per mm^3
        self.MAX_STEPS = self.STEPS_PER_MMCUBED*self.MAX_VOL # number of steps needed to fill pouch
        self.MIN_STEPS = 50
        # print(maxSteps)
        self.TIMESTEP = 0.01 # Inverse of sampling frequency on arduinos
        self.CABLE_SPEED_LIM = 50 # mm/s SET HIGH TO REMOVE FROM SYSTEM FOR NOW


        ###################################################################
        # Pulse generation
        ###################################################################
        #Arduino clock frequency
        self.CLOCK_FREQ = 16000000
        # Arduino prescalar value
        self.PRESCALER = 8
        # numBits = 16
        # OCR = np.linspace(0, 2**numBits, (2**numBits)+1)
        # f8 = CLOCK_FREQ/(PRESCALER*(OCR+1))
        self.MAX_FREQ = 2000
        self.TWO2_16 = 2**16


        ###################################################################
        # Lookup table
        ###################################################################
        # Lookup array of equally spaced theta values in interval 0 to pi/2
        self.NUM_POINTS = 10000
        self.THETA_VECT = np.linspace(0, mt.pi/2, self.NUM_POINTS)
        self.SPACING = (mt.pi/2)/(self.NUM_POINTS-1)

        # Lookup array of cable contractions/length changes.
        # Numpy uses unnormalised sinc function so divide by pi. Using sinc
        # avoids divide by zero errors returned when computing np.sin(x)/x
        self.CABLE_LOOKUP = self.L_0*(1 - np.sinc(self.THETA_VECT/mt.pi)) # Lc lookup
        # self.derivL = np.gradient(self.CABLE_LOOKUP, self.SPACING)

        # Avoid division by zero by prepending volLookup with zero.
        self.THETA_VECT_NO_ZERO = self.THETA_VECT[1:self.NUM_POINTS]
        self.VOL_LOOKUP = (self.THETA_VECT_NO_ZERO - np.cos(self.THETA_VECT_NO_ZERO)*np.sin(self.THETA_VECT_NO_ZERO))/(self.THETA_VECT_NO_ZERO**2)
        self.VOL_LOOKUP = np.insert(self.VOL_LOOKUP, 0, 0, axis=None)
        # self.derivV = np.gradient(self.VOL_LOOKUP, self.SPACING)

        # From pouch motors:
        # Add correction for when actuators are flat
        # P is pressure, Ce = 5.0e-6 Pa-1
            # d = Ce*P
            # cableLength*(1 + d*mt.pi/(mt.pi - 2)) - d
        # d/dt(sinc(t)) = (t*cos(t)-sin(t))/t**2
        # print(cableLookup)


        ###################################################################
        # Analytical approximation of Volume based on Contraction
        ###################################################################
        self.DIST_TO_CENT = (self.SIDE_LENGTH/2)/mt.cos(mt.pi/6)
        # Contraction  self.L_c = (self.SIDE_LENGTH - targetCable)/self.MA
        # where targetCable is target cable length from lin algebra
        self.MA = 2 # Mechanical advantage of pulleys
        if self.SIDE_LENGTH == 50:
            self.MAX_CABLE_DIST = 40.91    # 35.41 or 40.91
        elif self.SIDE_LENGTH == 40:
            self.MAX_CABLE_DIST = 35.41
        elif self.SIDE_LENGTH == 18.78:
            self.MAX_CABLE_DIST = 17.50
        #Initialise at centre
        self.L_c = (self.MAX_CABLE_DIST - self.DIST_TO_CENT)/self.MA
        # Store current value of contraction 
        self.cL_c = self.L_c
        self.MIN_CONTRACT = 0.1


        ###################################################################
        # End Effector and Entry Point Geometry
        ###################################################################
        # a (alpha) is yaw angle wrt global frame
        self.ALPHA_YAW = 0
        # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
        # For transformation from base orientation to conventional instrument frame, add further rotation
        # This is an input
        self.ROT_GLOB_INST = np.array([[mt.cos(self.ALPHA_YAW), -mt.sin(self.ALPHA_YAW), 0],\
            [mt.sin(self.ALPHA_YAW), mt.cos(self.ALPHA_YAW), 0],\
            [0, 0, 1]])

        # r (radius) is radius of end effector
        self.RAD_END = 0 # millimetres, possible because of rotating end effector
        # This matrix defines the three instrument attachment points wrt instrument frame
        self.ATTACH_POINTS = np.array([[self.RAD_END, -self.RAD_END, 0],\
            [-self.RAD_END, self.RAD_END, 0],\
            [0, self.RAD_END, 0]])
        # Z axis out of screen - conssitent with 'master' controller

        # Entry point array - global frame defined at bottom lhs corner (from behind instrument)
        # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
        self.ENTRY_POINTS = np.array([[-self.SIDE_LENGTH/2, -self.SIDE_LENGTH/2*mt.tan(mt.pi/6),  0],\
                                      [self.SIDE_LENGTH/2,  -self.SIDE_LENGTH/2*mt.tan(mt.pi/6),  0],\
                                      [0,                    self.SIDE_LENGTH*mt.tan(mt.pi/6),    0]])
        # print(self.ENTRY_POINTS)
        self.ENTRY_POINTS = np.transpose(self.ENTRY_POINTS)

        # Initialise matrix describing cable direction vectors:
        self.uCables = np.array([[-0.5*mt.tan(mt.pi/3), -0.5, 0],\
            [0.5*mt.tan(mt.pi/3), -0.5, 0],\
            [0, 1, 0]])
        self.uCables = np.transpose(self.uCables)

        
        ###################################################################
        # Extension/Retraction Geometry
        ###################################################################
        # Point that the shaft rotates around - COR of universal joint
        self.CONT_ARC_S = 15 # mm continuum joint arc length
        self.LEVER_BASE_Z = -25.0 # checked with sldasm
        # self.LEVER_POINT = np.array([0.5*self.SIDE_LENGTH,\
        #     0.5*self.SIDE_LENGTH*mt.tan(mt.pi/6),\
        #     self.LEVER_BASE_Z])
        # self.LEVER_POINT = np.array([0,\
        #                              self.SIDE_LENGTH*mt.tan(mt.pi/6),\
        #                              self.LEVER_BASE_Z])
        self.LEVER_POINT = np.array([0,\
                                     0,\
                                     self.LEVER_BASE_Z])
        self.E12 = self.ENTRY_POINTS[:, 1] - self.ENTRY_POINTS[:, 0]
        self.E13 = self.ENTRY_POINTS[:, 2] - self.ENTRY_POINTS[:, 0]
        self.N_CROSS = np.cross(self.E12, self.E13)
        # Normal of end effector / parallel mechanism plane:
        self.N_PLANE = self.N_CROSS/la.norm(self.N_CROSS)

        self.SHAFT_LENGTH_UJ = 50 # mm
        self.SHAFT_LENGTH = self.SHAFT_LENGTH_UJ - self.CONT_ARC_S # mm     SHAFT_LENGTH_UJ - CONT_ARC

        # Set limits on shaft extension
        # self.minShaftExt = self.SHAFT_LENGTH + 1
        self.MIN_EXTEND = 0.5 # mm
        self.MAX_EXTEND = 40 # mm
        # Set limit when curvature of continuum joint is assumed zero
        self.MIN_CONT_RAD = 0.1 # mm
        # Max angle that hydraulic motors can have is:
        # print(self.volToAngle(self.MAX_VOL))


    def intersect(self, tDesX, tDesY, tExt):
        # CONTINUUM JOINT
        # Transform coords to reference base of continuum joint, not parallel mech centre
        tDesZ = tExt + self.LEVER_BASE_Z
        #Desired point P_des
        P_des = np.array([tDesX, tDesY, tDesZ])
        # print("P_des: ", P_des)

        # Project desired point onto plane coincidnet with lever base point and 
        # parallel to entry point plane
        proj_lever_base = [tDesX, tDesY, self.LEVER_POINT[2]]
        # Distance to centre of projected point
        proj_rad = la.norm(proj_lever_base - self.LEVER_POINT)

        # Use continuum joint model if curvature of continuum part significant
        # Else use universal joint model
        if (proj_rad > self.MIN_CONT_RAD): 
            x_bend_plane = proj_rad
            y_bend_plane = tDesZ
            # angle between x axis at base point and projected point
            ang_around_shaft = mt.atan2(tDesY - self.LEVER_POINT[1], tDesX - self.LEVER_POINT[0])

            # Find angle of continuum joint
            # Use theta polynomial from Taylor approximations of sin, cos, & tan:
            theta_poly = [x_bend_plane/3, -(self.CONT_ARC_S/2 - y_bend_plane), -x_bend_plane]
            root_theta = np.roots(theta_poly)
            # print("root_theta: ", root_theta)
            theta_approx = float(root_theta[root_theta > 0])
            # print("theta_approx: ", theta_approx)

            # Continuum joint radius
            cont_rad = self.CONT_ARC_S/theta_approx

            # Continuum joint tip in bending plane (2D) frame of reference:
            shaft_start_xL = float(cont_rad*(1 - mt.cos(theta_approx)))
            shaft_start_yL = float(cont_rad*(mt.sin(theta_approx)))

            # Transform from bending plane to 3D workspace
            conty = np.array([shaft_start_xL, 0, shaft_start_yL])
            # print("conty: ", conty)

            # Rotate around z axis:
            cont_Rz = np.array([[mt.cos(ang_around_shaft), -mt.sin(ang_around_shaft), 0],\
                    [mt.sin(ang_around_shaft),  mt.cos(ang_around_shaft), 0],\
                    [0,                0,               1]])

            # print("Rz: ", cont_Rz)
            # This is tip of continuum joint
            conty_glob_0 = np.matmul(cont_Rz, conty)
            # print("Zero ", conty_glob_0)
            conty_glob = np.transpose(conty_glob_0) + self.LEVER_POINT
            # print("Glob: ", conty_glob) # SHOULD BE 3 X 1

            # Now find distance between this point and desired point,
            # subtract fixed shaft length to find prismatic length.
            L_Pri = la.norm(P_des - conty_glob) - self.SHAFT_LENGTH

            # Find where line between conty_glob and desired point P_des
            # intersects the entry point trangle:
            u_Cont = (P_des - conty_glob)/la.norm(P_des - conty_glob); 

        else: # Assume shaft is a UJ connected to lever point base
            baseToPoint = la.norm(P_des - self.LEVER_POINT)
            L_Pri = abs(baseToPoint) - self.SHAFT_LENGTH_UJ
            # Find where line between conty_glob and desired point P_des
            # intersects the entry point trangle:
            u_Cont = (P_des - self.LEVER_POINT)/baseToPoint
            theta_approx = 0
            ang_around_shaft = 0

        # print("u_cont: ", u_Cont)
        # How far along u_Cont the POI lies, starting from desired point P_des
        dist_to_POI = np.dot((self.ENTRY_POINTS[:, 0] - P_des), self.N_PLANE)/np.dot(u_Cont, self.N_PLANE)
        # print("dist_to_POI: ", dist_to_POI)
        POI_Cont = P_des + dist_to_POI*u_Cont

        # print("POI and Pri: ", POI_Cont[0], POI_Cont[1], L_Pri)

        # Impose contraction range
        if (L_Pri < self.MIN_EXTEND):
            L_Pri = self.MIN_EXTEND
        elif (L_Pri > self.MAX_EXTEND):
            L_Pri = self.MAX_EXTEND

        return POI_Cont[0], POI_Cont[1], L_Pri, theta_approx, ang_around_shaft



    def cableLengths(self, cX, cY, tX, tY):
        """
        Function finds cable lengths in mm from entry points to end effector
        given an input point (x, y) in mm.
        Also returns pseudoinverse of Jacobian for new point.
        Jacobian is transpose of pose dependent structure matrix
        structure matrix.
        e.g. [cableL, cableR, cableT, Jplus] = cableLengths(15, 8.6603)
        """
        # currPos is the current position on plane, targPos is target position
        currPos = np.array([[cX], [cY], [0]])
        targPos = np.array([[tX], [tY], [0]])

        # Find cable attachment points in global frame
        currPos_GI = np.dot(self.ROT_GLOB_INST, self.ATTACH_POINTS) + currPos
        targPos_GI = np.dot(self.ROT_GLOB_INST, self.ATTACH_POINTS) + targPos
        # print(currPos_GI)
        
        # Result is 3x3 matrix of vectors in global frame pointing from attachment points to
        # entry points, norms of columns are cable lengths
        cL = self.ENTRY_POINTS - currPos_GI
        tL = self.ENTRY_POINTS - targPos_GI
        # Current cable lengths for current Jaco calculation
        cLhsCable = la.norm(cL[:,0])
        cRhsCable = la.norm(cL[:,1])
        cTopCable = la.norm(cL[:,2])
        # Target cable lengths calulation
        tLhsCable = la.norm(tL[:,0])
        tRhsCable = la.norm(tL[:,1])
        tTopCable = la.norm(tL[:,2])
        # print(cL)

        # Compute the structure matrix A from cable unit vectors and cable attachment points 
        # in global frame, currPos_GI
        # Find cable unit vectors
        uLhs = cL[:,0]/cLhsCable    ### FILTER OUT ZERO LENGTH ERRORS?
        uRhs = cL[:,1]/cRhsCable
        uTop = cL[:,2]/cTopCable
        self.uCables = np.array([uLhs, uRhs, uTop])
        self.uCables = np.transpose(self.uCables)
        # print(uCables)
        # Find cross products of cable unit vectors and attachment points
        pCrossU1 = np.cross(currPos_GI[:,0], uLhs)
        pCrossU2 = np.cross(currPos_GI[:,1], uRhs)
        pCrossU3 = np.cross(currPos_GI[:,2], uTop)
        pCrossU = np.array([pCrossU1, pCrossU2, pCrossU3])
        pCrossU = np.transpose(pCrossU)
        # print(pCrossU)

        # Construct Jacobian from transpose of structure matrix
        cJacobian = np.concatenate((self.uCables, pCrossU), axis = 0)
        # Use only top two rows
        Jplus = np.linalg.pinv(cJacobian[0:2,:])
        # print(cJacobian[0:2,:])
        # rank(A) # Check for singular configuration
        # If rank(A) < number controlled DOFs, A is singular

        return tLhsCable, tRhsCable, tTopCable, cJacobian, Jplus



    def length2Vol (self, currentCable, targetCable):
        """
        Function returns required volume in ml in muscle to reach desired length,
        the piston speed required, as well as displacement from 0 position on pump.
        Uses lookup table/interpolation to approximate inverse of sin(theta)/theta
        for required length, given contractedL = flatL times sin(theta)/theta
        e.g. length2Vol(17.32, 18)
        """
        # Find contraction of actuator, filtering zeros:
        if targetCable < self.MAX_CABLE_DIST:
            self.L_c = (self.MAX_CABLE_DIST - targetCable)/self.MA
            # print(self.L_c)
        else:
            self.L_c = self.MIN_CONTRACT
        # Similar for current contraction
        self.cL_c = (self.MAX_CABLE_DIST - currentCable)/self.MA
        
        # Rate of change of cable length
        cableSpeed = (self.cL_c - self.cL_c)/self.TIMESTEP

        # Use Taylor approximation of theta and sub into 
        # theta-dependent part of volume equation:
        thetaApprox = abs(mt.sqrt(6*(self.L_c/1000)/(self.L_0/1000)))
        normV = thetaApprox*(thetaApprox**4 - 18*thetaApprox**2 + 96)/144
        # Multiply theta-dependent part with geometry-dependent part:
        volume = normV*self.FACT_V
        volComp = volume*self.VOL_FACTOR - self.DEAD_VOL

        # Find distance syringe pump has to move to reach desired volume
        lengthSyringe = volume/self.A_SYRINGE # A_sYRINGE is in mm^2
        lenComp = volComp/self.A_SYRINGE

        # Find desired position of stepper:
        stepCountUncomp = round(lengthSyringe*self.STEPS_PER_MM)
        stepCountComp = round(lenComp*self.STEPS_PER_MM)

        # Find discretised actuator length actuator actually commanded to go to:
        # lengthDisc = stepCountComp/self.STEPS_PER_MM
        # volDisc = lengthDisc*self.A_SYRINGE
        # normVDisc = volDisc/self.FACT_V
        # angleDisc = np.interp(normVDisc, self.VOL_LOOKUP, self.THETA_VECT)
        # LDisc = self.L_0*mt.sin(angleDisc)/angleDisc
        # LcDisc = self.L_0 - LDisc
        # print(Lc, LcDisc)
        # Convert from mm^3 to ml
        # volume = volume / 1000
        return volComp, cableSpeed, stepCountComp, self.L_c, thetaApprox


    def volToAngle(self, volume):
        desiredAngle = 360*(volume/self.A_SYRINGE)/self.LEAD
        return desiredAngle
    

    def volRate(self, cVol, cCable, tCable):
        """
        Makes linear approximation of volume rate.
        Returns volume rate in mm^3/s, syringe speed in mm/s, pulse frequency 
        in Hz. Current and target volumes and syringe displacements are also returned.
        """
        # USE CABLE SPEED AND INITIAL CABLE LENGTH AS INPUT?
        # Find current and target volume and displacement of syringe
        # [cV, cD] = length2Vol(cCable)
        [tVol, tSpeed, stepNo, LcDisc, angleDisc] = self.length2Vol(cCable, tCable)
        # print(tVol, tSpeed, stepNo, LcDisc, angleDisc)
        # if stepNo > self.MAX_STEPS*self.VOL_FACTOR:
        #     stepNo = self.MAX_STEPS*self.VOL_FACTOR
        # if stepNo < self.MIN_STEPS:
        #     stepNo = self.MIN_STEPS
        # Calculate linear approximation of volume rate:
        volDiff = tVol-cVol
        vDot = (volDiff)/self.TIMESTEP #timeSecs  # m^3/s
        dDot = (vDot/self.A_SYRINGE) # speed of syringe piston in m/s
        fStep = self.STEPS_PER_MM*dDot # stepper pulse frequency

        return tVol, vDot, dDot, fStep, stepNo, tSpeed, LcDisc, angleDisc



    def freqScale(self, fL, fR, fT, fE):
        """
        Returns output compare register values for use in interrupts.
        If any frequency exceeds stepper MAX_FREQ then all are scaled
        down to preserve velocity vector direction.
        """
        fList = np.array([fL, fR, fT])
        fAbs = np.absolute(fList)
        fRound = np.array([0, 0, 0])
        fRoundE = 0
        # Find OCR and scale if any of the frequency values is non-zero
        if np.any(fList):
            fSign = np.sign(fList)
            fMax = np.amax(fAbs)
            # If largest frequency is above MAX_FREQ then scale
            if fMax > self.MAX_FREQ:
                fFact = self.MAX_FREQ/fMax
                fScaled = fFact*fAbs
                fScaled = fScaled*fSign
                fList = fScaled
                # Else use unscaled frequencies
            fRound = np.around(self.TIMESTEP*fList*fSign)
            fRound = fRound*fSign
            fRound = np.int_(fRound)

        fSignE = np.sign(fE)
        if abs(fE) > self.MAX_FREQ:
            fE = self.MAX_FREQ
        fRoundE = round(abs(self.TIMESTEP*fE))
        fRoundE = int(fRoundE*fSignE)

        return fRound[0], fRound[1], fRound[2], fRoundE
    



    def volRateScale(self, tVolL, tVolR, tVolT, cVolL, cVolR, cVolT,):
        """
        Returns scaled target volumes 
        """
        tVolList = np.array([tVolL, tVolR, tVolT])
        cVolList = np.array([cVolL, cVolR, cVolT])
        volDiff = tVolList-cVolList
        volRate = (volDiff)/self.TIMESTEP #timeSecs  # m^3/s
        volAbs = np.absolute(tVolList)

        # Find OCR and scale if any of the frequency values is non-zero
        if np.any(volRate):
            volSign = np.sign(volRate)
            volMax = np.amax(volAbs)
            # If largest frequency is above MAX_FREQ then scale
            if volMax > self.MAX_VOL_RATE:
                volFact = self.MAX_VOL_RATE/volMax
                volScaled = volFact*volAbs
                volScaled = volScaled*volSign
                volChange = volScaled*self.TIMESTEP
                tVolList = cVolList + volChange
                # Else use unscaled frequencies
        return tVolList[0], tVolList[1], tVolList[2]



    def cableSpeeds (self, cX, cY, tX, tY, Jaco, JacoPlus):
        """
        Returns required cable length change rates to reach target
        from current point within primary sampling period.
        tX is target X, cX is current X.
        Desired speed in X and Y found by multiplying difference 
        by sampling frequency of primary. JacoPlus is pseudoinverse of
        Jacobian at a given point.
        """
        timeSecs = self.TIMESTEP

        # TARGET POINT, CURRENT POINT, TARGET SPEED ARE INPUTS
        # USED TO FIND CABLE LENGTHS AND RATE OF CABLE LENGTH CHANGE

        diffX = tX - cX
        tVx =  diffX/timeSecs
        diffY = tY - cY
        tVy = diffY/timeSecs
        vInput = np.array([[tVx],[tVy]])

        # IK premultiplying input end effector velocity vector with pseudoinverse J
        # to yield joint velocities.
        cableRates = np.dot(JacoPlus, vInput)
        absRates = np.absolute(cableRates)
        vSign = np.sign(cableRates)
        vMax = np.amax(absRates)
        # If any cable speed is non-zero
        if np.any(cableRates):
            if vMax > self.CABLE_SPEED_LIM:
                vFact = self.CABLE_SPEED_LIM/vMax
                vScaled = vFact*absRates
                vScaled = vScaled*vSign
                cableRates = vScaled
        # Resultant cable speeds 
        lhsSpeed = cableRates[0]
        rhsSpeed = cableRates[1]
        topSpeed = cableRates[2]
        # print(cableRates)

        # Find scaled velocity in Euclidean space by pre-multiplying joint velocities with J:
        vEuclid = np.dot(Jaco,cableRates)
        vEuclidXY = vEuclid[0:2]
        tVx = vEuclidXY[0]
        tVy = vEuclidXY[1]
        dirAngle = np.arctan2(tVy, tVx)
        perpAngle = dirAngle
        if dirAngle < 0:
            dirAngle = 2*np.pi + dirAngle
            perpAngle = dirAngle + np.pi/2
            if perpAngle > np.pi*2:
                perpAngle = perpAngle - 2*np.pi
        perpAngle = float(perpAngle)

        vMagScaled = mt.sqrt(tVx**2 + tVy**2)
        # print(vEuclidXY)

        # Find actual position after scaled movement:
        if np.any(vEuclidXY): # If tVx or tVy is non-zero
            if (mt.sqrt(diffX**2 + diffY**2) != 0):
                # The two lines below need better non-divide-by-zero
                actX = cX + ((diffX*vMagScaled)/mt.sqrt(diffX**2 + diffY**2))*timeSecs
                actY = cY + ((diffY*vMagScaled)/mt.sqrt(diffX**2 + diffY**2))*timeSecs
            else:
                actX = cX
                actY = cY
        else:
            actX = cX
            actY = cY
        # print(actX, actY)
        

        return lhsSpeed, rhsSpeed, topSpeed, actX, actY, perpAngle


    def collisionAngle(self, dL, dR, dT, conL, conR, conT):
        # In the case of no contact:
        if not(conL) and not(conR) and not(conT):
            collAngle = None
        else:
            derivRef = max(np.absolute(np.array([dL, dR, dT])))
            derRatL = dL/derivRef
            derRatR = dR/derivRef
            derRatT = dT/derivRef
            relTensions = np.array([derRatL, derRatR, derRatT])
            tensionVects = self.uCables*relTensions
            resultant = np.array([np.sum(tensionVects[0,:]), np.sum(tensionVects[1,:]), np.sum(tensionVects[2,:])])
            resDir = resultant/la.norm(resultant)
            resDirX = resDir[0]
            resDirY = resDir[1]
            collAngle = (180/np.pi)*np.arctan2(resDirY, resDirX)

        return collAngle



    def cableError (self, idealX, idealY, targetL, targetR, targetT, targetP, opX, opY, opZ):
        # est variables are based on optitrack data

        # Optitrack data uses different frame of reference so convert
        # ADD SIDELENGTH/2 AND SIDELENGTH/2 * SIN(PI/6)
        realX = -opZ + self.OFFSET_X
        realY = opY + self.OFFSET_Y
        realZ = opX
         
        # Based on estimated tip position in 3D space, find point of intersection (POI)
        # of shaft with parallel mech plane
        [estPOIX, estPOIY, estP, inclin, azimuth] = self.intersect(realX, realY, realZ) 
        # print("X, Y, P   : ", estPOIX, estPOIY, estP)
        # print("Position, local: ", realX, realY, realZ)

        # Estimate the true cable lengths based on vision system
        [estL, estR, estT, cJaco, cJpinv] = self.cableLengths(idealX, idealY, estPOIX, estPOIY)
        # print("Est cables: ", estL, estR, estT, estP)
       
        # Error between observed and open-loop targets
        errCableL =  estL - targetL
        errCableR = estR - targetR
        errCableT = estT - targetT
        errPrism = estP - targetP

        return errCableL, errCableR, errCableT, errPrism
    





WINDOW_SIZE = 10

class forceDetector:

    def __init__(self):

        self.pressArray = np.zeros(WINDOW_SIZE)
        self.pressMed = np.median(self.pressArray)
        self.pressMedians = np.zeros(3)

        self.pressMedPrev = self.pressMed
        self.presMedPrevPrev = self.pressMed

        self.derivThresh = 2000
        self.deriv2Thresh = 3000

        self.conDetected = False


    def newPressMed(self, newPress):

        self.pressArray = np.roll(self.pressArray, 1)
        self.pressArray[0] = newPress
        
        self.pressMedians = np.roll(self.pressMedians, 1)
        self.pressMedians[0] = np.median(self.pressArray)

        self.presMedPrevPrev = self.pressMedPrev
        self.pressMedPrev = self.pressMed
        self.pressMed = np.median(self.pressArray)

        return self.pressMed


    def derivPress(self, timeStep = None):
        # realTimeStep = 0.01 #(newTime - prevTime)/1000
        # # Prevent div by zero errors / limit noise?
        # if realTimeStep <= 0.001:
        #     realTimeStep = 0.001
        if timeStep is None:
            timeStep = 0.01
        # pressDiff = self.pressMed - self.pressMedPrev

        # Centred finite difference is for previous timestep
        centredDeriv = (self.pressMedians[0] - self.pressMedians[2])/2*timeStep
        secondDeriv = (self.pressMedians[0] - 2*self.pressMedians[1] + self.pressMedians[2])/(timeStep**2)

        # deriv = pressDiff/realTimeStep
        # deriv2 = deriv/realTimeStep
        # Check derivatives against heuristically determined threshold values to determine contact:
        # self.conDetected = (abs(deriv) > self.derivThresh) & (abs(deriv2) > self.deriv2Thresh)
        self.conDetected = (abs(centredDeriv) > self.derivThresh) & (abs(secondDeriv) > self.deriv2Thresh)
        # 1 means actuator in tension, -1 means compression, 0 means no contact
        self.conDetected = self.conDetected*np.sign(centredDeriv)
        return self.conDetected, centredDeriv, secondDeriv


if __name__ == "__main__":
    forceD = forceDetector()
    for x in range(4*WINDOW_SIZE):
        if (x < WINDOW_SIZE + 10):
            forceD.newPressMed(x)
        else:
            forceD.newPressMed(x*(-1)**x)

        [contactFlag, cDeriv, c2Deriv] = forceD.derivPress()

        print("Window of pressures: ", forceD.pressArray)
        print("Last three medians:  ", forceD.pressMedians)
        print("Derivatives:         ", cDeriv, c2Deriv)
