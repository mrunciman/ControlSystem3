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
        self.trackSock = NatNetClient()
        self.trackSock.set_client_address(optionsDict["clientAddress"])
        self.trackSock.set_server_address(optionsDict["serverAddress"])
        self.trackSock.set_use_multicast(optionsDict["use_multicast"])
        if self.trackSock.connected() is False:
            print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
            try:
                sys.exit(2)
            except SystemExit:
                print("...")
            finally:
                print("exiting")

        self.trackSock.new_frame_listener = self.receive_new_frame
        self.trackSock.rigid_body_listener = self.receive_rigid_body_frame

        is_running = self.trackSock.run()
        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        
        # self.trackSock.setblocking(False)
        # self.trackSock.connect(self.trackSock)
        # print(self.trackSock)
        self.packet = []
        self.trackData = []
        self.markerData = []





    def readSocket(self):
        # self.trackData = self.trackSock.recv(MAX_PACKETSIZE)
        self.trackData.send_request(socket, self.NAT_REQUEST_FRAMEOFDATA, command_str, address)
        print(self.trackData)
        self.packet = unpack(self.trackData, version=self.version)
        if type(self.packet) is SenderData:
            self.version = self.packet.natnet_version
        self.marker1 = self.packet.labeled_markers[0].position
        self.marker2 = self.packet.labeled_markers[1].position
        self.markerData.append([self.marker1] + [self.marker2])
        # self.marker3 = self.packet.labeled_markers[2].position
        # print("M1: ", self.marker1, "M2: ", self.marker2, "M3: ", self.marker3)
        # return self.marker1, self.marker2, self.marker3
        return self.marker1, self.marker2

    def optiSave(self):
        with open(fileName, 'a', newline='') as optiLog:
            optiLog2 = csv.writer(optiLog)
            for i in range(len( self.markerData)):
                optiLog2.writerow( self.markerData[i])
        return

    def closeSocket(self):
        self.trackSock.shutdown() # NatNetClient method
        self.trackSock.close()

    ######################################################################
    # Not part of class
    #####################################################################
    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(self, data_dict):
        order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                    "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += data_dict[key] + " "
                out_string+="/"
                self.trackData.append(out_string)
            print(out_string)

    # This is a callback function that gets connected to the NatNet client. 
    # It is called once per rigid body per frame
    def receive_rigid_body_frame(self, new_id, position, rotation ):
        pass
        #print( "Received frame for rigid body", new_id )
        #print( "Received frame for rigid body", new_id," ",position," ",rotation )
