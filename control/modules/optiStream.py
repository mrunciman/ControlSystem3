import csv
import os
import time
import sys
import numpy as np
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


    def optiConnect(self):
        is_running = self.trackSock.run()
        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        return is_running


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
        for i in range (0, len(rigid_body_list)):
            [id, pos, rot] = rigid_body_list[i]
            [alpha, beta, gamma] = self.quat_to_euler(rot[0], rot[1], rot[2], rot[3])
            rowData += [id, pos[0], pos[1], pos[2], alpha, beta, gamma, rot[0], rot[1], rot[2], rot[3]]
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




    def receive_rigid_body_frame(self, new_id, position, rotation):

        [alpha, beta, gamma] = self.quat_to_euler(rotation[0], rotation[1], rotation[2], rotation[3])
        rbDataLocal = [new_id, position, alpha, beta, gamma]
        # self.rigidData.append([new_id] + [position] + [alpha] + [beta] + [gamma])
        # print("Rigid Body from MoCap: ", rbDataLocal)
        # rbDataLocal2 = self.trackSock.rigid_body_data_out
        # print("Rigid Body from MoCap: ", rbDataLocal2)


def tip_position(self, robotFrame, instrumentFrame):
    # Method to get tip pose with respect to robot base



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