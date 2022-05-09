import csv
import os
import time
import sys
from modules import NatNetClient
from modules import DataDescriptions
from modules import MoCapData

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/opti/optiTrack " + logTime + ".csv"
fileName = os.path.join(parent, relative)


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
        self.pluggedIn = False
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
        self.trackSock.rigid_body_listener = self.receive_rigid_body_frame

        # self.trackSock.data_out = mocap_data = MoCapData.MarkerSetData() # See NatNetClient.py
        self.markerData = []#self.trackSock.data_out.marker_set_data.unlabeled_markers.marker_pos_list 
        self.timeStamp = []#self.trackSock.data_out.suffix_data.timestamp


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


    def optiSave(self):
        with open(fileName, 'a', newline='') as optiLog:
            optiLog2 = csv.writer(optiLog)
            for i in range(len( self.markerData)):
                optiLog2.writerow( self.markerData[i])
        return


    def optiClose(self):
        self.trackSock.shutdown() # NatNetClient method
        # self.trackSock.close()


    #####################################################################
    # These are callback functions that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(self, data_dict):
        markerDataLocal = self.trackSock.data_out.unlabeled_markers.marker_pos_list
        # Append all data into rows of a list instead of list of tuples:
        self.markerData.append([self.trackSock.time_stamp] + [item for t in markerDataLocal for item in t])
        # print(self.markerData)
        self.timeStamp = self.trackSock.time_stamp
        # self.markerData.insert(self.timeStamp)
        # print(self.timeStamp)



    # This is a callback function that gets connected to the NatNet client. 
    # It is called once per rigid body per frame
    def receive_rigid_body_frame(self, new_id, position, rotation ):
        pass
        #print( "Received frame for rigid body", new_id )
        #print( "Received frame for rigid body", new_id," ",position," ",rotation )
