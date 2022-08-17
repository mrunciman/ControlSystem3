import csv
import os
import time
import sys
import numpy as np
from numpy import linalg as la
from modules import NatNetClient
from modules import DataDescriptions
from modules import MoCapData

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/opti/optiTrack " + logTime + ".csv"
fileName = os.path.join(parent, relative)
with open(fileName, mode ='w', newline='') as optiLog0: 
    optiLog1 = csv.writer(optiLog0)
    optiLog1.writerow(['time stamp', 'cumulative time', 'ID', 'X', 'Y', 'Z', 'Y angle', 'Z angle', 'X angle', 'w', 'i', 'j', 'k',\
        'ID', 'X', 'Y', 'Z', 'Y angle', 'Z angle', 'X angle', 'i', 'j', 'k', 'w'])

class optiTracker:

    def __init__(self):
        self.marker1 = (0, 0, 0)
        self.marker2 = (0, 0, 0)
        self.marker3 = (0, 0, 0)
        # self.version = (2, 9, 0, 0)  # NatNet version to use
        optionsDict = {}
        optionsDict["clientAddress"] = "127.0.0.1"
        optionsDict["serverAddress"] = "127.0.0.1"
        optionsDict["use_multicast"] = True
        self.trackSock = NatNetClient.NatNetClient()
        self.trackSock.set_client_address(optionsDict["clientAddress"])
        self.trackSock.set_server_address(optionsDict["serverAddress"])
        self.trackSock.set_use_multicast(optionsDict["use_multicast"])
        # self.pluggedIn = False
        # if self.trackSock.connected() is False:
        #     print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        #     try:
        #         sys.exit(2)
        #     except SystemExit:
        #         print("...")
        #     finally:
        #         print("exiting")
        # else:
        #     self.pluggedIn = True
        self.pluggedIn = True

        self.trackSock.new_frame_listener = self.receive_new_frame
        self.trackSock.rigid_body_listener = self.receive_rigid_bodies

        # self.trackSock.data_out = mocap_data = MoCapData.MarkerSetData() # See NatNetClient.py
        self.rigidData = [] 
        # self.rigidData = # self.trackSock.rigid_body_data_out.rigid_body_list
        self.markerData = []#self.trackSock.data_out.marker_set_data.unlabeled_markers.marker_pos_list 
        self.timeStamp = time.time() #self.trackSock.data_out.suffix_data.timestamp
        self.timeRunning = 0

        self.frameData = []
        self.T_Robot_Inst = np.array([]) # poor style but works

        self.M_TO_MM = 1000

    def optiConnect(self, useRigidBodies):
        running = self.trackSock.run()
        # if not running:
        #     print("ERROR: Could not start streaming client.")
        #     try:
        #         sys.exit(1)
        #     except SystemExit:
        #         print("...")
        #     finally:
        #         print("exiting")

        if useRigidBodies == True:
            T_Rob_Inst = self.tip_pose()
            # print(T_Rob_Inst)
            if np.any(T_Rob_Inst) != True:
                print("Optitrack not connected!")
                optiTrackConnected = False
                # self.optiClose()
            else:
                optiTrackConnected = True
        
        else:
            
            if self.markerData == []:
                time.sleep(0.1)
                if self.markerData == []:
                    optiTrackConnected = False
                else:
                    optiTrackConnected = True
            else:
                optiTrackConnected = True

        return optiTrackConnected


    def optiSave(self, dataToSave):
        with open(fileName, 'a', newline='') as optiLog2:
            optiLog3 = csv.writer(optiLog2)
            for i in range(len(dataToSave)):
                optiLog3.writerow(dataToSave[i])
        return


    def optiClose(self):
        self.trackSock.shutdown() # NatNetClient method
        # self.trackSock.close()


    #####################################################################
    # These are callback functions that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(self, data_dict):
        markerDataLocal = self.trackSock.data_out.unlabeled_markers.marker_pos_list
        self.timeStamp = time.time() 
        self.timeRunning += 1/120

        # Append all data into rows of a list instead of list of tuples:
        self.markerData.append([self.timeStamp] + [self.timeRunning] + [item for t in markerDataLocal for item in t])
        # print(self.markerData)


    # This is a callback function that gets connected to the NatNet client. 
    # It is called once per rigid body per frame
    def receive_rigid_bodies(self, rigid_body_list):
        self.timeStamp = time.time()
        rowData = []
        self.frameData = rigid_body_list
        for i in range (0, len(rigid_body_list)):
            [id, pos, quat] = rigid_body_list[i]
            [alpha, beta, gamma] = self.quat_to_euler(quat[0], quat[1], quat[2], quat[3])
            rowData += [id, pos[0], pos[1], pos[2], alpha, beta, gamma, quat[0], quat[1], quat[2], quat[3]]
        # [alpha, beta, gamma] = self.quat_to_euler(rotation[0], rotation[1], rotation[2], rotation[3])
        self.rigidData.append([self.timeStamp] + [self.timeRunning] + rowData)


    def quat_to_euler(self, x, y, z, w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))
        # X = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))
        # Y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))
        # Z = np.arctan2(t3, t4)

        return X, Y, Z 


    def quat_to_rot_matrix(self, i, j, k, w): 
        #requires quaternions in  w, i, j, k order
        q0 = w
        q1 = i
        q2 = j
        q3 = k
            
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1 
        r01 = 2 * (q1 * q2 - q0 * q3) 
        r02 = 2 * (q1 * q3 + q0 * q2) 
            
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3) 
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1) 
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2) 
        r21 = 2 * (q2 * q3 + q0 * q1) 
        r22 = 2 * (q0 * q0 + q3 * q3) - 1 
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],\
                               [r10, r11, r12],\
                               [r20, r21, r22]])
        
        return rot_matrix


    # def receive_rigid_body_frame(self, new_id, position, rotation):

    #     [alpha, beta, gamma] = self.quat_to_euler(rotation[0], rotation[1], rotation[2], rotation[3])
    #     rbDataLocal = [new_id, position, alpha, beta, gamma]
        # self.rigidData.append([new_id] + [position] + [alpha] + [beta] + [gamma])
        # print("Rigid Body from MoCap: ", rbDataLocal)
        # rbDataLocal2 = self.trackSock.rigid_body_data_out
        # print("Rigid Body from MoCap: ", rbDataLocal2)

    # def buildMatrix(self, R, T):
    #     tMatrixI = np.block([[rotMatrixI[0, :], self.M_TO_MM*posI[0]],\
    #                     [rotMatrixI[1, :], self.M_TO_MM*posI[1]],\
    #                     [rotMatrixI[2, :], self.M_TO_MM*posI[2]],\
    #                     [0, 0, 0, 1]])
    #     return tMatrixI

    def get_frames(self):
        if len(self.frameData) == 2:
            [idI, posI, quatI] = self.frameData[0]
            # posI = np.array(posI)
            rotMatrixI = self.quat_to_rot_matrix(quatI[0], quatI[1], quatI[2], quatI[3])
            tMatrixI = np.block([[rotMatrixI[0, :], self.M_TO_MM*posI[0]],\
                                 [rotMatrixI[1, :], self.M_TO_MM*posI[1]],\
                                 [rotMatrixI[2, :], self.M_TO_MM*posI[2]],\
                                 [0, 0, 0, 1]])
            # print(tMatrixI)

            [idR, posR, quatR] = self.frameData[1]
            rotMatrixR = self.quat_to_rot_matrix(quatR[0], quatR[1], quatR[2], quatR[3])
            tMatrixR = np.block([[rotMatrixR[0, :], self.M_TO_MM*posR[0]],\
                                 [rotMatrixR[1, :], self.M_TO_MM*posR[1]],\
                                 [rotMatrixR[2, :], self.M_TO_MM*posR[2]],\
                                 [0, 0, 0, 1]])

            if posI[0] < posR[0]:
                tempR = tMatrixR
                tMatrixR = tMatrixI
                tMatrixI = tempR
            # print(tMatrixR)
        else:
            tMatrixI = np.zeros((4,4))
            tMatrixR = np.zeros((4,4))


        
        return tMatrixI, tMatrixR


    def tip_pose(self,T_W_Inst_camera=None):
        # Method to get tip pose with respect to robot base
        [T_W_Inst, T_W_Rob] = self.get_frames()
        T_Rob_W = np.linalg.pinv(T_W_Rob)
        T_Rob_Inst = np.dot(T_Rob_W, T_W_Inst)
        if T_W_Inst_camera is not None:
            # print("T_W_Inst Optitrack estimate: ", T_W_Inst)
            # print("T_W_Inst Vision estimate: ", T_W_Inst_camera)
            T_Rob_Inst_camera =  np.dot(T_Rob_W, T_W_Inst_camera)
            return T_Rob_Inst, T_Rob_Inst_camera
        else:
            return T_Rob_Inst

    



# # Robot base
# qx = -0.003272728
# qy = 0.003644174
# qz = -0.013302799
# qw = -0.999899566

# # Robot tip
# qx = -0.069562741	
# qy = 0.00613268	
# qz = 0.064799614
# qw = 0.995451927


# [ X, Y, Z ] = optiStream.quat_to_euler(qx,qy,qz,qw)
# print( X, Y, Z )