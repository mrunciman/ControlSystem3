#!/usr/bin/env python3

import socket
import time
# import subprocess
# from ctypes import *



class omniStreamer():
    def __init__(self):
        self.server_addr = ('localhost', 8888)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.omniX = 0.0 # mm
        self.omniY = 0.0
        self.omniZ = 0.0

    def connectOmni(self):
        # problem with this is that it wiats for program to terminate, which never happens
        # omniServer = subprocess.run([r"C:\OpenHaptics\Developer\3.5.0\examples\HD\console\HelloHapticDevice\Win32\Debug\HelloHapticDevice.exe"],\
        #     check=True, capture_output=True, text=True)
        # print("stdout:", omniServer.stdout)
        # print("stdout:", omniServer.stderr)

        try:
            self.sock.connect(self.server_addr)
            # self.sock.setblocking(0)
            self.sock.settimeout(0.024)
            print("Connected to {:s}".format(repr(self.server_addr)))
        except AttributeError as ae:
            print("Error creating the socket: {}".format(ae))

    def getOmniCoords(self):
        try:
            handshake = b'1'
            self.sock.send(handshake)
            data = self.sock.recv(512)
            # print(len(data))
            stringdata = data.decode('utf-8')
            numdata = stringdata.split(";")
            numdata = numdata[0:-1] # Remove empty entry at end due to split
            # print(numdata)
            # numdata has an empty character appended to it as the last element
            # if (len(numdata) > 3): # Need a better filter here - start and end bits?
            count = 1
            for i in numdata:
                if i == "S":
                    if len(numdata)-count >= 4:
                        if numdata[count+3] == "E": #index is count minus one (then plus x for relevant digit), for 0 indexing
                            self.omniX = float(numdata[count])
                            self.omniY = float(numdata[count+1])
                            self.omniZ = float(numdata[count+2])
                            # print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
                            break
                        else:
                            # print(numdata[count+3])
                            break # just ignore and wait for nex one
                            # raise ValueError('Somehow the endByte is not what was expected') # Somehow the endByte isn't what was expected
                else:
                    count+=1

            # if numdata[0] == ord("S"):
            #     self.omniX = float(numdata[0])
            #     self.omniY = float(numdata[1])
            #     self.omniZ = float(numdata[2])
                # print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
        except socket.timeout:
            pass
        except socket.error as se:
            print("Exception on socket: {}".format(se))
            print("Closing socket")
            self.sock.close()
        # return self.omniX, self.omniY, self.omniZ

    def omniMap(self):#, xFromOmni, yFromOmni, zFromOmni):
        #Calibrated position in inkwell:
        #  x:  0.00000 , y:  -65.51071 , z:  -88.11420
        # self.omniX = self.omniX + 220
        # self.omniY = self.omniY + 110
        # self.omniZ = -(self.omniZ - 125)
        # if (self.omniX < 0):
        #     self.omniX = 0
        # if (self.omniY < 0):
        #     self.omniY = 0
        # if (self.omniZ < 0):
        #     self.omniZ = 0


        xMapped = -(self.omniX*((46-(-46))/440)) + 9.455 # abs just for test
        yMapped = (self.omniY*((55-(-35))/310))  + 5.4591
        zMapped = -(self.omniZ*((75-30)/215))*2 + 10 # Omni direction is opposite to real direction
        # if (xMapped < 0):
        #     xMapped = 0
        # if (yMapped < 0):
        #     yMapped = 0
        if (zMapped < 0):
            zMapped = 0
        if (zMapped > 38.5):
            zMapped = 38.5
        # print("x: ", xMapped, ", y: ", yMapped, ", z: ", zMapped)
        return xMapped, yMapped, zMapped
        
        

    def omniClose(self):
        self.sock.close()


# if __name__ == "__main__":
#     main()