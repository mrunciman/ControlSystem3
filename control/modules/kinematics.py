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
        # self.L_0 = 30
        self.L_0 = 72
        self.MIN_CONTRACT = 0.2
        self.MAX_CONTRACT = 12.5
        self.STROKE = self.MAX_CONTRACT - self.MIN_CONTRACT


        self.D_s = 14 # Flat section of actuator
        self.D_t = 24 # Total width of actuator
        self.D_c = (self.D_t - self.D_s)/2 # Width of each individual conic end
        # Number of length subdivisions
        self.NUM_L = 6
        self.FACT_V = ((self.L_0**2)/self.NUM_L)*(self.D_c/3 + self.D_s/2)
        # Syringe cross sectional area, diameter = 12.5 mm
        self.SYRINGE_RADIUS = 12.5/2
        self.A_SYRINGE = mt.pi*(self.SYRINGE_RADIUS**2) # mm^2
        # Real volume calc: there are numLs beams of length L0/numLs
        # self.FACT_V = ((self.ACT_WIDTH/1000)*(self.L_0/1000)**2)/(2*self.NUM_L)
        self.M3_to_MM3 = 1e9
        self.VOL_FACTOR = 0.95 # Maximum value of 0.775 # Ratio of real volume to theoretical volume
        self.CAL_FACTOR = 0.005 # % of max volume still in actuator after calibration
        self.FACT_ANG = 1
        self.MAX_VOL = self.FACT_V*((mt.pi/2*self.FACT_ANG) - \
            mt.cos((mt.pi/2*self.FACT_ANG))*mt.sin((mt.pi/2*self.FACT_ANG)))/((mt.pi/2*self.FACT_ANG)**2)
        print("Max volume of actuator: ", self.MAX_VOL)
        self.DEAD_VOL = self.CAL_FACTOR*self.MAX_VOL
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
        # self.NUM_POINTS = 10000
        # self.THETA_VECT = np.linspace(0, mt.pi/2, self.NUM_POINTS)
        # self.SPACING = (mt.pi/2)/(self.NUM_POINTS-1)

        # # Lookup array of cable contractions/length changes.
        # # Numpy uses unnormalised sinc function so divide by pi. Using sinc
        # # avoids divide by zero errors returned when computing np.sin(x)/x
        # self.CABLE_LOOKUP = self.L_0*(1 - np.sinc(self.THETA_VECT/mt.pi)) # Lc lookup
        # # self.derivL = np.gradient(self.CABLE_LOOKUP, self.SPACING)

        # # Avoid division by zero by prepending volLookup with zero.
        # self.THETA_VECT_NO_ZERO = self.THETA_VECT[1:self.NUM_POINTS]
        # self.VOL_LOOKUP = (self.THETA_VECT_NO_ZERO - np.cos(self.THETA_VECT_NO_ZERO)*np.sin(self.THETA_VECT_NO_ZERO))/(self.THETA_VECT_NO_ZERO**2)
        # self.VOL_LOOKUP = np.insert(self.VOL_LOOKUP, 0, 0, axis=None)
        # self.derivV = np.gradient(self.VOL_LOOKUP, self.SPACING)

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
        self.ALPHA_YAW = 0
        # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
        # For transformation from base orientation to conventional instrument frame, add further rotation
        # This is an input
        self.ROT_GLOB_INST = np.array([[mt.cos(self.ALPHA_YAW), -mt.sin(self.ALPHA_YAW), 0],\
                                       [mt.sin(self.ALPHA_YAW),  mt.cos(self.ALPHA_YAW), 0],\
                                       [0,                      0,                       1]])

        # r (radius) is radius of end effector
        self.RAD_END = 3.0/2 # millimetres, zero is possible because of rotating shaft coupling
        # This matrix defines the three instrument attachment points wrt instrument frame, rows are right, left, top
        self.ATTACH_POINTS = np.array([[self.RAD_END*mt.cos(210 * mt.pi/180), self.RAD_END*mt.sin(210 * mt.pi/180), 0],\
                                       [self.RAD_END*mt.cos(330 * mt.pi/180), self.RAD_END*mt.sin(330 * mt.pi/180), 0],\
                                       [self.RAD_END*mt.cos(90 * mt.pi/180),  self.RAD_END*mt.sin(90 * mt.pi/180),  0]])
        self.ATTACH_POINTS = np.transpose(self.ATTACH_POINTS)

        self.attach_points_rot = self.ATTACH_POINTS
        self.attach_points_cont = self.attach_points_rot
        # Z axis out of screen - conssitent with 'master' controller

        # Entry point array - global frame defined at bottom lhs corner (from behind instrument)
        # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
        self.ENTRY_POINTS = np.array([[-self.SIDE_LENGTH/2, -self.SIDE_LENGTH/2*mt.tan(mt.pi/6),  0],\
                                      [ self.SIDE_LENGTH/2, -self.SIDE_LENGTH/2*mt.tan(mt.pi/6),  0],\
                                      [ 0,                   self.SIDE_LENGTH*mt.tan(mt.pi/6),    0]])
        # print(self.ENTRY_POINTS)
        self.ENTRY_POINTS = np.transpose(self.ENTRY_POINTS)

        # Initialise matrix describing cable direction vectors:
        self.uCables = np.array([[-0.5*mt.tan(mt.pi/3), -0.5, 0],\
            [0.5*mt.tan(mt.pi/3), -0.5, 0],\
            [0, 1, 0]])
        self.uCables = np.transpose(self.uCables)


        ###################################################################
        # Analytical approximation of Volume based on Contraction
        ###################################################################
        # Contraction  self.L_c = (self.SIDE_LENGTH - targetCable)/self.MECH_ADV
        # where targetCable is target cable length from lin algebra
        # This part depends on how we define where the cable is at minimum or maximum stroke. E.g:
        # If the cable is at an opposing corner when the actuator is flat, the maximum distance is the side length of the triangle
        # If the cable is at the same corner when the actuator is at maximum contraction, the max distance is the stoke times the mech. adv.
        self.MECH_ADV = 3 # Mechanical advantage of pulleys

        self.MIN_CABLE = 0 #see Matlab WorkspaceOptimisation_InVivo.m
        self.MAX_CABLE_DIST = (self.STROKE*self.MECH_ADV) + self.MIN_CABLE # see Sizing.sldprt
        # print("Max cable dist ", self.MAX_CABLE_DIST)
        self.RANGE = self.STROKE*self.MECH_ADV #self.MAX_CABLE_DIST - self.MIN_CABLE

        #Initialise at centre
        self.L_c = ((self.MAX_CABLE_DIST - self.MAX_CABLE_DIST/2)/self.MECH_ADV) #self.STROKE * ((self.RANGE - (L_c_centre - self.MIN_CABLE))/self.RANGE)
        # Store current value of contraction 
        self.cL_c = self.L_c

        
        ###################################################################
        # Extension/Retraction Geometry
        ###################################################################
        # Point that the shaft rotates around - COR of universal joint
        self.CONT_ARC_S = 10 # mm continuum joint arc length

        self.LEVER_BASE_Z = -10 # checked with calipers

        self.LEVER_POINT = np.array([0,\
                                     self.SIDE_LENGTH*mt.tan(mt.pi/6) - 10,\
                                     self.LEVER_BASE_Z])
        # print(self.LEVER_POINT)
        self.E12 = self.ENTRY_POINTS[:, 1] - self.ENTRY_POINTS[:, 0]
        self.E13 = self.ENTRY_POINTS[:, 2] - self.ENTRY_POINTS[:, 0]
        self.N_CROSS = np.cross(self.E12, self.E13)
        # Normal of end effector / parallel mechanism plane:
        self.N_PLANE = self.N_CROSS/la.norm(self.N_CROSS)

        self.SHAFT_LENGTH = 21 # mm
        self.SHAFT_LENGTH_UJ = self.SHAFT_LENGTH + self.CONT_ARC_S # mm

        # Set limits on shaft extension
        # self.minShaftExt = self.SHAFT_LENGTH + 1
        self.MIN_EXTEND = 0.5 # mm
        self.MAX_EXTEND = 55 # mm
        self.initialPrismLength = 0
        # Set limit when curvature of continuum joint is assumed zero
        self.MIN_CONT_RAD = 0.1 # mm
        # Max angle that hydraulic motors can have is:
        # print(self.volToAngle(self.MAX_VOL))




        ###################################################################
        # Gantry robot geometry and limits
        ###################################################################
        # Limits on angle change of torque coil / rotary axis
        self.MIN_ROTARY = -300.0   # degrees 
        self.MAX_ROTARY =  300.0   # degrees 

        # Limits on wrist angle
        self.MIN_WRIST_ANGLE = 0   # degrees 
        self.MAX_WRIST_ANGLE = 90  # degrees 
        # Geometry of wrist motor
        self.HYPOT_TIP = 5.62      # mm
        self.TIP_CUTAWAY_WIDTH = 2.34 # mm
        self.NUM_SUBSECTIONS = 6   # number of cutaways
        self.RADIUS_SPOOL = 5      # mm
        self.THETA_REST = 2*mt.asin((self.TIP_CUTAWAY_WIDTH/2)/self.HYPOT_TIP)
        self.WRIST_THETA_0 = mt.radians(24)
        self.WRIST_D_0 = 2*self.HYPOT_TIP*mt.sin(self.WRIST_THETA_0/2)

        # Limits on how far instrument can be extended
        self.MIN_TOOL_EXT = 5      # mm
        self.MAX_TOOL_EXT = 150     # mm
        # Geometry of tool extensionm motor 
        self.TOOL_EXT_ROLLER_RADIUS = 5 #mm


    def intersect(self, tDesX, tDesY, tExt):
        # CONTINUUM JOINT
        # Transform coords to reference base of continuum joint, not parallel mech centre
        tZShifted = tExt - self.LEVER_BASE_Z # Use vertically shifted value to keep y_bend_plane positive
        #Desired point P_des
        P_des = np.array([tDesX, tDesY, tExt]) # Where parallel mechanism is at z = 0
        azimuth = mt.atan2(tDesX - self.LEVER_POINT[0], tExt - self.LEVER_POINT[2])

        # Project desired point onto plane coincident with lever base point and 
        # parallel to entry point plane
        proj_lever_base = [tDesX, tDesY, self.LEVER_POINT[2]]
        # print("Old pror_lever_base: ", proj_lever_base)
        # Distance to centre of projected point
        proj_rad = la.norm(proj_lever_base - self.LEVER_POINT)

        # Use continuum joint model if curvature of continuum part significant
        # Else use universal joint model
        if (proj_rad > self.MIN_CONT_RAD): 
            x_bend_plane = proj_rad
            y_bend_plane = tZShifted # Use vertically shifted value to keep y_bend_plane positive
            # angle between x axis at base point and projected point
            ang_around_shaft = mt.atan2(tDesY - self.LEVER_POINT[1], tDesX - self.LEVER_POINT[0])

            # Find angle of continuum joint
            # Use theta polynomial from Taylor approximations of sin, cos, & tan:
            theta_poly = [x_bend_plane/3, -(self.CONT_ARC_S/2 - y_bend_plane), -x_bend_plane]
            root_theta = np.roots(theta_poly)
            # print("root_theta: ", root_theta)
            theta_approx = float(root_theta[root_theta > 0])
            # print("around_shaft: ", ang_around_shaft)
            # print("theta_approx: ", theta_approx)
            # Continuum joint radius
            cont_rad = self.CONT_ARC_S/theta_approx

            # Continuum joint tip in bending plane (2D) frame of reference:
            shaft_start_xL = float(cont_rad*(1 - mt.cos(theta_approx)))
            shaft_start_yL = float(cont_rad*(mt.sin(theta_approx)))

        else: # Assume shaft is a UJ connected to lever point base
            theta_approx = 0
            ang_around_shaft = 0

            shaft_start_xL = 0
            shaft_start_yL = self.CONT_ARC_S
        # print("Old continuum tip 2D:", shaft_start_xL, shaft_start_yL)
        # print("Old angRS: ", ang_around_shaft)

        rotOutOfPlane_yaw = np.array([[mt.cos(-ang_around_shaft), -mt.sin(-ang_around_shaft), 0],\
                                        [mt.sin(-ang_around_shaft),  mt.cos(-ang_around_shaft), 0],\
                                        [0,                         0,                        1]])
        
        rotOutOfPlane_pitch = np.array([[ mt.cos(theta_approx), 0, mt.sin(theta_approx)],\
                                        [ 0, 1, 0],\
                                        [-mt.sin(theta_approx), 0, mt.cos(theta_approx)]])
        
        rotOutOfPlane_yawRev = np.array([[mt.cos(ang_around_shaft), -mt.sin(ang_around_shaft), 0],\
                                    [mt.sin(ang_around_shaft),  mt.cos(ang_around_shaft), 0],\
                                    [0,                         0,                        1]])
        
        attach_points_int0 = np.dot(rotOutOfPlane_yaw, self.ATTACH_POINTS)
        attach_points_int1 = np.dot(rotOutOfPlane_pitch, attach_points_int0)
        self.attach_points_rot = np.dot(rotOutOfPlane_yawRev, attach_points_int1)

        # Transform from bending plane to 3D workspace
        conty = np.array([shaft_start_xL, 0, shaft_start_yL])
        # print("Old conty: ", conty)

        # Rotate around z axis:
        cont_Rz = np.array([[mt.cos(ang_around_shaft), -mt.sin(ang_around_shaft), 0],\
                [mt.sin(ang_around_shaft),  mt.cos(ang_around_shaft), 0],\
                [0,                0,               1]])

        # print("Rz: ", cont_Rz)
        # This is tip of continuum joint
        conty_glob_0 = np.matmul(cont_Rz, conty)
        # print("Zero ", conty_glob_0)
        # Shift tip coords back in z direction as parallel mech is on z = 0 plane 
        conty_glob = np.transpose(conty_glob_0) + self.LEVER_POINT 
        # print("Old Glob: ", conty_glob) # SHOULD BE 3 X 1

        # Now find distance between this point and desired point,
        # subtract fixed shaft length to find prismatic length.
        L_Pri = la.norm(P_des - conty_glob) - self.SHAFT_LENGTH

        # Find where line between conty_glob and desired point P_des
        # intersects the entry point trangle:
        u_Cont = (P_des - conty_glob)/la.norm(P_des - conty_glob)
        # print("Old u_Cont: ", u_Cont)
        L_Pri = L_Pri*np.sign(u_Cont[2])
            

        # print("u_cont: ", u_Cont)
        # How far along u_Cont the POI lies, starting from desired point P_des
        distToPlane = np.dot((self.ENTRY_POINTS[:, 0] - P_des), self.N_PLANE)/np.dot(u_Cont, self.N_PLANE)
        # Impose contraction range
        if (L_Pri < self.MIN_EXTEND):
            L_Pri = self.MIN_EXTEND
        elif (L_Pri > self.MAX_EXTEND):
            L_Pri = self.MAX_EXTEND

        # print(L_Pri)
        # Find end of shaft assembly
        POI_Cont = P_des - L_Pri*u_Cont
        # print("Old shaft tip: ", POI_Cont)
        # Find POI of shaft with parallel plane
        POI_Plane = P_des + distToPlane*u_Cont
        # print("Old POI_Plane: ", POI_Plane)
        # print("Old P_des: ", P_des)


        return POI_Cont[0], POI_Cont[1], L_Pri, theta_approx, ang_around_shaft, azimuth, POI_Cont, POI_Plane
    

    def intersectPolar(self, thetaIn, azimuthIn, prismIn ):
        #Given a theta and phi(azimuth) angle from the controller, find the angle around the shaft to get the continuum tip
        # Relative to global coordinate frame, order of operations is pitch first, then roll by theta
        # New z axis coords can make up direction vector u_Cont, which can be projected onto global frame to find angle_round_shaft
        # u_Cont points from continuum tip to P_des, reached by extending by prismIn from continuum tip

        theta_approx = thetaIn #float(root_theta[root_theta > 0])
        # azimuthIn = np.pi/2

        #These will be used for rotation respect to global frame
        dirMatrix_roll = np.array([[1, 0, 0],\
                                        [0,  mt.cos(theta_approx), -mt.sin(theta_approx)],\
                                        [0, mt.sin(theta_approx), mt.cos(theta_approx)]])
        
        dirMatrix_pitch = np.array([[ mt.cos(azimuthIn), 0, mt.sin(azimuthIn)],\
                                        [ 0, 1, 0],\
                                        [-mt.sin(azimuthIn), 0, mt.cos(azimuthIn)]])
        
        dirMatrix = np.dot(dirMatrix_roll, dirMatrix_pitch)
        u_Cont_Combined = dirMatrix[:,2]#dirMatrix_roll[:,2]
        u_Cont_Combined = u_Cont_Combined/la.norm(u_Cont_Combined)
        u_Cont_theta = dirMatrix_roll[:,2]
        u_Cont_theta = u_Cont_theta/la.norm(u_Cont_theta)

        # vector = np.array([1, 1, 0])
        
        # # Normalize vectors
        # vector_norm = np.linalg.norm(vector)
        # if vector_norm == 0:
        #     raise ValueError("Input vector must not be zero.")
        
        # vector_normalized = vector / vector_norm
        z_axis = np.array([0, 0, 1])
        # Dot product and angle
        dot_product = np.dot(u_Cont_Combined, z_axis)
        # Clamp dot product to valid acos range [-1, 1] to prevent numerical issues
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)

        dirMatrix_roll_2 = np.array([[1, 0, 0],\
                                [0,  mt.cos(angle_rad), -mt.sin(angle_rad)],\
                                [0, mt.sin(angle_rad), mt.cos(angle_rad)]])
        u_Cont_theta_2 = dirMatrix_roll_2[:,2]
        # print("Angle to Z: ", angle_rad)


        # Use continuum joint model if curvature of continuum part significant
        # Else use universal joint model
        # print("Theta: ", theta_approx)
        # print("Azimuth:" , azimuthIn)
        if (abs(angle_rad) > 0.0001): 
            # Continuum joint radius
            cont_rad = self.CONT_ARC_S/angle_rad

            # Continuum joint tip in bending plane (2D) frame of reference:
            shaft_start_xL = float(cont_rad*(1 - mt.cos(angle_rad)))
            shaft_start_yL = float(cont_rad*(mt.sin(angle_rad)))

            # Transform from bending plane to 3D workspace
            # conty = np.array([0, -shaft_start_xL, shaft_start_yL])
            # print("New conty: ", conty)

        # elif (abs(azimuthIn) > 0.0001): # Assume shaft is a UJ connected to lever point base
        #     # Continuum joint radius
        #     cont_rad = self.CONT_ARC_S/azimuthIn

        #     # Continuum joint tip in bending plane (2D) frame of reference:
        #     shaft_start_xL = float(cont_rad*(1 - mt.cos(azimuthIn)))
        #     shaft_start_yL = float(cont_rad*(mt.sin(azimuthIn)))
        #     # Transform from bending plane to 3D workspace
        #     conty = np.array([0, -shaft_start_xL, shaft_start_yL])
        #     print("New conty - Azimuth: ", conty)

        else:
            shaft_start_xL = 0
            shaft_start_yL = self.CONT_ARC_S
            # Transform from bending plane to 3D workspace
        conty = np.array([0, -shaft_start_xL, shaft_start_yL])
        # print("New conty: ", conty)
        # print("New continuum tip 2D:", shaft_start_xL, shaft_start_yL)


        # Set prismatic joint
        L_Pri = prismIn
        # print(L_Pri)

        # # Impose contraction range
        if (L_Pri < self.MIN_EXTEND):
            L_Pri = self.MIN_EXTEND
        elif (L_Pri > self.MAX_EXTEND):
            L_Pri = self.MAX_EXTEND


        # Find end of shaft assembly
        POI_Cont_0 = conty + self.SHAFT_LENGTH*u_Cont_theta_2
        # print("Intermediate shaft tip: ", POI_Cont_0)
        P_des_0 = (POI_Cont_0 + L_Pri*u_Cont_theta_2) 
        # print("intermediate P_Des: ", P_des_0)
        #What rotation around shaft gets you the desired azimuth angle?
        # Desired z coordinate stays the same compared to POI_Cont_0
        zd = P_des_0[2] + self.LEVER_POINT[2]
        # print("Desired z: ", zd)
        # Knowing the azimuth angle we can find the desired x coordinate

        # Then the projected distance yd_0 from the continuum base to the desired yd coord can be found
        yd_0 = abs(P_des_0[1])
        # print("yd_0: ", yd_0)
        #What rotation around shaft gets you the desired azimuth angle?
        xd = -np.tan(azimuthIn)*(zd - self.LEVER_POINT[2]) + self.LEVER_POINT[0]
        # print("Desired x: ", xd)
        if yd_0 < 0.0001:    #((abs(theta_approx) < 0.001) and (abs(azimuthIn) < 0.001)):
            ang_around_shaft = 0
            xd = 0
            # print("Ang forced to zero 2")
        else:
            if (abs(xd/yd_0) <= 1):
                ang_around_shaft = mt.acos(xd/yd_0) # fix this?
            else:
                ang_around_shaft = mt.acos(np.clip(xd/yd_0, -1.0, 1.0))
                # ang_around_shaft = 0
                # print("Ang forced to zero 1", xd/yd_0)

        # print("Ang_round 1: ", ang_around_shaft)


        # The final coordinate of the desired point is then yd
        if theta_approx < 0:
            yd = yd_0*mt.sin(ang_around_shaft) + self.LEVER_POINT[1]
            # print("Theta negative")
        else:
            yd = -yd_0*mt.sin(ang_around_shaft) + self.LEVER_POINT[1]
            # print("Theta positive")
        # yd =  yd_0*mt.sin(ang_around_shaft) + self.LEVER_POINT[1]
        ang_around_shaft = mt.atan2(yd - self.LEVER_POINT[1], xd - self.LEVER_POINT[0]) # recalculate to handle polarity
        # print("Ang_round 2: ", ang_around_shaft)
        # print("Desired y: ", yd)


        rotOutOfPlane_yaw = np.array([[mt.cos(-ang_around_shaft), -mt.sin(-ang_around_shaft), 0],\
                                        [mt.sin(-ang_around_shaft),  mt.cos(-ang_around_shaft), 0],\
                                        [0,                         0,                        1]])
        
        rotOutOfPlane_pitch = np.array([[ mt.cos(theta_approx), 0, mt.sin(theta_approx)],\
                                        [ 0, 1, 0],\
                                        [-mt.sin(theta_approx), 0, mt.cos(theta_approx)]])
        
        rotOutOfPlane_yawRev = np.array([[mt.cos(ang_around_shaft), -mt.sin(ang_around_shaft), 0],\
                                    [mt.sin(ang_around_shaft),  mt.cos(ang_around_shaft), 0],\
                                    [0,                         0,                        1]])
            
        attach_points_int0 = np.dot(rotOutOfPlane_yaw, self.ATTACH_POINTS)
        attach_points_int1 = np.dot(rotOutOfPlane_pitch, attach_points_int0)
        self.attach_points_rot = np.dot(rotOutOfPlane_yawRev, attach_points_int1)

        conty_0 = np.array([shaft_start_xL, 0, shaft_start_yL])
        # Rotate continuum tip back around z axis:
        cont_Rz = rotOutOfPlane_yawRev 
        # print("Rz: ", cont_Rz)
        # This is tip of continuum joint
        conty_glob_0 = np.matmul(cont_Rz, conty_0)
        # print("Zero ", conty_glob_0)
        # Shift tip coords back in z direction as parallel mech is on z = 0 plane 
        conty_glob = np.transpose(conty_glob_0) + self.LEVER_POINT 
        # print("New Glob: ", conty_glob) # SHOULD BE 3 X 1

        P_des = [xd, yd, zd]
        u_Cont = (P_des - conty_glob)/la.norm(P_des - conty_glob)
        # print("New angRS: ", ang_around_shaft)
        # print("New P_des: ", P_des, '\n')
        L_Pri = L_Pri*np.sign(u_Cont[2])
        POI_Cont = P_des - L_Pri*u_Cont
        # print("New shaft tip: ", POI_Cont)

        # How far along u_Cont the POI lies, starting from desired point P_des
        distToPlane = np.dot((self.ENTRY_POINTS[:, 0] - P_des), self.N_PLANE)/np.dot(u_Cont, self.N_PLANE)
        # Find POI of shaft with parallel plane
        POI_Plane = P_des + distToPlane*u_Cont
        # print("New POI_Plane: ", POI_Plane)

        return POI_Cont[0], POI_Cont[1], L_Pri, theta_approx, ang_around_shaft, azimuthIn, POI_Cont, POI_Plane, P_des



    def cableLengths(self, cX, cY, tX, tY, c_conty_glob = None, t_conty_glob = None):
        """
        Function finds cable lengths in mm from entry points to end effector
        given an input point (x, y) in mm.
        Also returns pseudoinverse of Jacobian for new point.
        Jacobian is transpose of pose dependent structure matrix
        structure matrix.
        e.g. [cableL, cableR, cableT, Jplus] = cableLengths(15, 8.6603)
        """
        # currPos is the current position on plane, targPos is target position
        if c_conty_glob is not None:
            currPos = np.array([[c_conty_glob[0]], [c_conty_glob[1]], [c_conty_glob[2]]])
            targPos = np.array([[t_conty_glob[0]], [t_conty_glob[1]], [t_conty_glob[2]]])
        else:
            currPos = np.array([[cX], [cY], [0]])
            targPos = np.array([[tX], [tY], [0]])

        # Find cable attachment points in global frame
        currPos_GI = np.dot(self.ROT_GLOB_INST, self.attach_points_rot) + currPos
        targPos_GI = np.dot(self.ROT_GLOB_INST, self.attach_points_rot) + targPos
        self.attach_points_cont = targPos_GI
        # print("TargPos: ", targPos_GI)
        
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
        # print("Cable lengths: ", tLhsCable, tRhsCable, tTopCable)

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

        self.cL_c = self.L_c
        # Find contraction of actuator, filtering zeros:
        #TODO Fix this for smooth and continuous function
        if targetCable <= (self.MAX_CABLE_DIST - self.MIN_CONTRACT):
            if targetCable > self.MIN_CABLE:
                # self.L_c = self.STROKE * (self.RANGE - (targetCable - self.MIN_CABLE))/self.RANGE
                self.L_c = ((self.MAX_CABLE_DIST - targetCable)/self.MECH_ADV)
            else:
                self.L_c = self.MAX_CONTRACT
        else:
            self.L_c = self.MIN_CONTRACT
        # print(self.L_c)
        # Similar for current contraction
        # self.cL_c = (self.MAX_CABLE_DIST - currentCable)/self.MECH_ADV
        # self.cL_c = self.STROKE * (self.RANGE - (currentCable - self.MIN_CABLE))/self.RANGE
        
        # Rate of change of cable length
        cableSpeed = (self.cL_c - self.cL_c)/self.TIMESTEP

        # Use Taylor approximation of theta and sub into 
        # theta-dependent part of volume equation:
        thetaApprox = abs(mt.sqrt(6*(self.L_c/1000)/(self.L_0/1000)))
        normV = thetaApprox*(thetaApprox**4 - 18*thetaApprox**2 + 96)/144
        # Multiply theta-dependent part with geometry-dependent part:
        # SUB_VOL_FACTOR = 0.9
        # self.VOL_FACTOR = SUB_VOL_FACTOR*(1 + 0.025*(volume/self.MAX_VOL))
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
            if volMax > self.MAX_VOL:
                volFact = self.MAX_VOL/volMax
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
    

    # self.STEPS_PER_REV = 200
    # self.MICROSTEPS = 16
    # self.MICROSTEPS_PRI = 4
    # self.LEAD = 8
    # self.STEPS_PER_MM = (self.STEPS_PER_REV*self.MICROSTEPS)/(self.LEAD) # steps per mm
    # self.STEPS_PER_MM_PRI = (self.STEPS_PER_REV*self.MICROSTEPS_PRI)/(self.LEAD) # steps per mm


    def setAxialMotor(self, desAxialPos):
        # Impose contraction range
        if (desAxialPos < self.MIN_EXTEND):
            desAxialPos = self.MIN_EXTEND
        elif (desAxialPos > self.MAX_EXTEND):
            desAxialPos = self.MAX_EXTEND

        axialMotorAngle = 360.0*desAxialPos/self.LEAD
        axialMotorAngle = round(axialMotorAngle,2)
            
        return axialMotorAngle, desAxialPos


    def setRotaryMotor(self, desRotaryPos):
        # TODO define MIN and MAX ROTARY angles
        if (desRotaryPos < self.MIN_ROTARY):
            desRotaryPos = self.MIN_ROTARY
        elif (desRotaryPos > self.MAX_ROTARY):
            desRotaryPos = self.MAX_ROTARY

        rotaryMotorAngle = desRotaryPos
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
        graspMotorAngle = 360.0*desGraspPos/self.LEAD
        graspMotorAngle = round(graspMotorAngle,2)
        return graspMotorAngle, desGraspPos



WINDOW_SIZE = 20

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
        self.pressMed = round(np.median(self.pressArray), 2)

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
