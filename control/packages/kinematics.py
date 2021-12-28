"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

# sideLength = 18.911

class kineSolver:
    # sideLength0 = sideLength
    def __init__(self, triangleSide):

        ##############################################################
        # Structure and actuators
        ##############################################################
        # Define parameters of system:
        # Side length of equilateral triangle in mm
        self.sideLength = triangleSide
        # 'Flat' muscle length:
        self.L0 = 25/(1 - 2/mt.pi)
        # Excess length of cable between entry point and muscle, in mm
        # self.Lx = 10
        # Total length of cable in flat/resting state
        # self.Lt = self.L0 + self.Lx + self.sideLength
        # print(Lt)
        # Width of hydraulic muscle in mm
        self.muscleWidth = 30
        # Number of length subdivisions
        self.numLs = 3
        # Syringe cross sectional area, diameter = 26.5 mm
        self.As = mt.pi*(13.25**2) # mm^2
        # Real volume calc: there are numLs beams of length L0/numLs
        self.factV = (self.muscleWidth*(self.L0)**2)/(2*self.numLs)
        self.volFactor = 0.9024 #12.6195/15.066 # Ratio of real volume to theoretical volume
        self.calFactor = 0.01 # % of max vol still in actuator at calibration
        self.factAng = 1
        self.maxV = self.factV*((mt.pi/2*self.factAng) - \
            mt.cos((mt.pi/2*self.factAng))*mt.sin((mt.pi/2*self.factAng)))/((mt.pi/2*self.factAng)**2)
        # print(self.maxV)
        
        # For step count:
        # Mapping from step to 1 revolution = 200 steps
        # 32 microsteps per step, so 6400 steps per revolution
        # Set M0, M1, M2 to set microstep size
        # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
        # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
        #          = 200*4/8 = 100 steps/mm
        # For speed: pulses/mm * mm/s = pulses/s
        self.StepPerRev = 200
        self.Microsteps = 256
        self.Lead = 8
        self.stepsPMM = (self.StepPerRev*self.Microsteps)/(self.Lead) # steps per mm
        self.stepsPV = self.stepsPMM/self.As # Steps per mm^3
        self.maxSteps = self.stepsPV*self.maxV # number of steps needed to fill pouch
        self.minSteps = 500
        # print(maxSteps)
        self.timeStep = 6/125 # Inverse of sampling frequency on arduinos
        self.cableSpeedLim = 4 # mm/s

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
        self.MAX_FREQ = 1500
        self.TWO2_16 = 2**16

        ###################################################################
        # Lookup table
        ###################################################################
        # Lookup array of equally spaced theta values in interval 0 to pi/2
        self.numPoints = 10000
        self.theta = np.linspace(0, mt.pi/2, self.numPoints)
        self.spacing = (mt.pi/2)/(self.numPoints-1)
        # theta = np.linspace(((mt.pi/2)/numPoints), mt.pi/2, numPoints-1)
        # Lookup array of cable contractions/length changes.
        # Numpy uses unnormalised sinc function so divide by pi. Using sinc
        # avoids divide by zero errors returned when computing np.sin(x)/x
        self.cableLookup = self.L0*(1 - np.sinc(self.theta/mt.pi)) # Lc lookup
        self.derivL = np.gradient(self.cableLookup, self.spacing)
        # Avoid division by zero by prepending volLookup with zero.
        self.thetaNoZero = self.theta[1:self.numPoints]
        self.volLookup = (self.thetaNoZero - np.cos(self.thetaNoZero)*np.sin(self.thetaNoZero))/(self.thetaNoZero**2)
        self.volLookup = np.insert(self.volLookup, 0, 0, axis=None)
        self.derivV = np.gradient(self.volLookup, self.spacing)
        # From pouch motors:
        # Add correction for when actuators are flat
        # P is pressure, Ce = 5.0e-6 Pa-1
            # d = Ce*P
            # cableLength*(1 + d*mt.pi/(mt.pi - 2)) - d
        # d/dt(sinc(t)) = (t*cos(t)-sin(t))/t**2
        # print(cableLookup)

        ###################################################################
        # End Effector and Entry Point Geometry
        ###################################################################
        # a (alpha) is yaw angle wrt global frame
        self.a = 0
        # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
        # For transformation from base orientation to conventional instrument frame, add further rotation
        # This is an input
        self.RotGI = np.array([[mt.cos(self.a), -mt.sin(self.a), 0], [mt.sin(self.a), mt.cos(self.a), 0], [0, 0, 1]])

        # r (radius) is radius of end effector
        self.r = 0 # millimetres, possible because of rotating end effector
        # This matrix defines the three instrument attachment points wrt instrument frame
        self.attP = np.array([[self.r, -self.r, 0], [-self.r, self.r, 0], [0, self.r, 0]])
        # Z axis out of screen - conssitent with 'master' controller

        # Entry point array - global frame defined at bottom lhs corner (from behind instrument)
        # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
        self.E = np.array([[0, 0, 0], [self.sideLength, 0, 0], [0.5*self.sideLength, self.sideLength*mt.sin(mt.pi/3), 0]])
        self.E = np.transpose(self.E)
        # Matrix describing cable direction vectors:
        self.uCables = np.array([[-0.5*mt.tan(mt.pi/3), -0.5, 0], [0.5*mt.tan(mt.pi/3), -0.5, 0], [0, 1, 0]])
        self.uCables = np.transpose(self.uCables)

        ###################################################################
        # Extension/Retraction Geometry
        ###################################################################
        # Point that the shaft rotates around - COR of universal joint
        self.leverBaseZ = -30
        self.leverPoint = np.array([0.5*self.sideLength, 0.5*self.sideLength*mt.tan(mt.pi/6), self.leverBaseZ])
        self.E12 = self.E[:, 1] - self.E[:, 0]
        self.E13 = self.E[:, 2] - self.E[:, 0]
        self.nCross = np.cross(self.E12, self.E13)
        # Normal of end effector / parallel mechanism plane:
        self.nPlane = self.nCross/la.norm(self.nCross)

        self.shaftLength = 67.5 # mm
        # Set limits on shaft extension
        # self.minShaftExt = self.shaftLength + 1
        # self.maxShaftExt = self.shaftLength + 13.5 # Range for spring loaded muscle different
        self.minExtend = 0.5
        self.maxExtend = 38.5


    def intersect(self, tDesX, tDesY, tDesZ):
        #Transform coords to reference base of continuum joint, not parallel mech centre:
        tDesZ = tDesZ + self.leverBaseZ + self.shaftLength
        tExt = np.array([tDesX, tDesY, tDesZ]) # Point in 3D where tip should be

        # Find vector PrD from lever point to target point
        baseToPoint = la.norm(tExt - self.leverPoint)
        PrD = (tExt - self.leverPoint)/baseToPoint
        # How far along PrD the POI is, from desired point tExt
        d = np.dot((self.E[:, 0] - tExt), self.nPlane)/np.dot(PrD, self.nPlane)
        POI = tExt + d*PrD
        # print(POI, -d)

        LcE = abs(baseToPoint) - self.shaftLength
        # Impose contraction range
        if LcE < self.minExtend:
            LcE = self.minExtend
        elif LcE > self.maxExtend:
            LcE = self.maxExtend

        # THIS COMPENSATES FOR self.length2Vol WHERE CABLE IS CONSIDERED PART OF PARALLEL MECHANISM
        # tCableE = self.sideLength - LcE
        # stepsPrismatic = int(self.stepsPMM*LcE)

        return POI[0], POI[1], LcE



    def cableLengths(self, cX, cY, tX, tY):
        """
        Function finds cable lengths in mm from entry points to end effector
        given an input point (x, y) in mm.
        Also returns pseudoinverse of Jacobian for new point.
        Jacobian is transpose of pose dependent structure matrix
        structure matrix.
        e.g. [cableL, cableR, cableT, Jplus] = cableLengths(15, 8.6603)
        """
        # cP is the current position on plane, tP is target position
        cP = np.array([[cX], [cY], [0]])
        tP = np.array([[tX], [tY], [0]])

        # Find cable attachment points in global frame
        cPGI = np.dot(self.RotGI, self.attP) + cP
        tPGI = np.dot(self.RotGI, self.attP) + tP
        # print(cPGI)
        
        # Result is 3x3 matrix of vectors in global frame pointing from attachment points to
        # entry points, norms of columns are cable lengths
        cL = self.E - cPGI
        tL = self.E - tPGI
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
        # in global frame, cPGI
        # Find cable unit vectors
        u1 = cL[:,0]/cLhsCable    ### FILTER OUT ZERO LENGTH ERRORS
        u2 = cL[:,1]/cRhsCable
        u3 = cL[:,2]/cTopCable
        self.uCables = np.array([u1, u2, u3])
        self.uCables = np.transpose(self.uCables)
        # print(uCables)
        # Find cross products of cable unit vectors and attachment points
        pCrossU1 = np.cross(cPGI[:,0], u1)
        pCrossU2 = np.cross(cPGI[:,1], u2)
        pCrossU3 = np.cross(cPGI[:,2], u3)
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
        Lc = self.sideLength - targetCable
        Lc0 = self.sideLength - currentCable
        cableSpeed = (Lc-Lc0)/self.timeStep
        # Use lookup tables defined above to find theta angle based on input length
        angle = np.interp(Lc, self.cableLookup, self.theta)
        # Calculate normalised volume of a beam of arbitrary size
        if angle <= 0:
            angle = 0.1
        normV = (angle - mt.cos(angle)*mt.sin(angle))/(angle**2)    ### FILTER OUT ANGLE = 0 ERRORS
        # normComp = ((angle*self.factAng) - mt.cos((angle*self.factAng))*mt.sin((angle*self.factAng)))/((angle*self.factAng)**2)
        # print(angle)
        # Real volume calc: multiply theta-dependent part of 
        # volume by constant geometry-dependent factor
        # self.volFactor = 1 - 0.1*(angle/(mt.pi/2))
        volume = normV*self.factV
        volComp = volume*self.volFactor + self.calFactor*self.maxV 
        # volComp = normComp*self.factV

        # angle = np.interp(volume, volLookup, theta)
        # Find distance syringe pump has to move to reach desired volume
        lengthSyringe = volume/self.As
        lenComp = volComp/self.As
        # print(volume/1000)

        stepCountComp = round(lenComp*self.stepsPMM)
        stepCountUncomp = round(lengthSyringe*self.stepsPMM)

        # Find discretised actuator length actuator actually commanded to go to:
        lengthDisc = stepCountUncomp/self.stepsPMM
        volDisc = lengthDisc*self.As
        normVDisc = volDisc/self.factV
        angleDisc = np.interp(normVDisc, self.volLookup, self.theta)
        LDisc = self.L0*mt.sin(angleDisc)/angleDisc
        LcDisc = self.L0 - LDisc
        # print(Lc, LcDisc)
        # Convert from mm^3 to ml
        # volume = volume / 1000
        return volComp, cableSpeed, stepCountComp, LcDisc, angleDisc




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
        if stepNo > self.maxSteps*self.volFactor:
            stepNo = self.maxSteps*self.volFactor
        if stepNo < self.minSteps:
            stepNo = self.minSteps
        # Calculate linear approximation of volume rate:
        volDiff = tVol-cVol
        vDot = (volDiff)/self.timeStep #timeSecs  # mm^3/s
        dDot = (vDot/self.As) # mm/s

        # For step count:
        # Mapping from step to 1 revolution = 200 steps
        # Set M0, M1, M2 to set microstep size
        # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
        # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
        # For speed: pulses/mm * mm/s = pulses/s
        fStep = self.stepsPMM*dDot

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
            fRound = np.around(self.timeStep*fList*fSign)
            fRound = fRound*fSign
            fRound = np.int_(fRound)

        fSignE = np.sign(fE)
        if abs(fE) > self.MAX_FREQ:
            fE = self.MAX_FREQ
        fRoundE = round(abs(self.timeStep*fE))
        fRoundE = int(fRoundE*fSignE)

        return fRound[0], fRound[1], fRound[2], fRoundE



    def cableSpeeds (self, cX, cY, tX, tY, Jaco, JacoPlus, timeSecs):
        """
        Returns required cable length change rates to reach target
        from current point within primary sampling period.
        tX is target X, cX is current X.
        Desired speed in X and Y found by multiplying difference 
        by sampling frequency of primary. JacoPlus is pseudoinverse of
        Jacobian at a given point.
        """
        timeSecs = self.timeStep

        # TARGET POINT, CURRENT POINT, TARGET SPEED ARE INPUTS
        # USED TO FIND CABLE LENGTHS AND RATE OF CABLE LENGTH CHANGE
        diffX = tX - cX
        #############################################
        # Testing to see how it looks with fixed time step, in this case 0.01 s for a 100 Hz loop
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
            if vMax > self.cableSpeedLim:
                vFact = self.cableSpeedLim/vMax
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
        

        return lhsSpeed, rhsSpeed, topSpeed, actX, actY


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
