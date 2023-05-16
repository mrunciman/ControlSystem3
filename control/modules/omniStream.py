import socket
import time
import subprocess
import numpy as np
import os
# from ctypes import *
np.set_printoptions(suppress=True, precision = 2)

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
relative = "modules\HelloHapticDevice.exe"
fileName = os.path.join(parent, relative)
# fileName should be: 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/modules/HelloHapticDevice.exe'


class omniStreamer():
    def __init__(self):
        self.server_addr = ('localhost', 8888)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.omniX = 0.0 # mm
        self.omniY = 0.0
        self.omniZ = 0.0
        self.tMatrix = []


    def connectOmni(self):
        # problem with this was that it waited for program to terminate, which never happens, but stdin=None, stdout=None, stderr=None argumetns sorted this
        omniServer = subprocess.run(fileName,\
            check=True, capture_output=True, stdin=None, stdout=None, stderr=None)
        try:
            self.sock.connect(self.server_addr)
            # self.sock.setblocking(0)
            self.sock.settimeout(0.024)
            print("Connected to {:s}".format(repr(self.server_addr)))
            print(self.sock)
            return True
        except AttributeError as ae:
            # print("Error creating the socket: {}".format(ae))
            return False
        except OSError as os:
            # print("Error creating socket: {}".format(os))
            return False


    def getOmniCoords(self):
        try:
            handshake = b'1'
            self.sock.send(handshake)
            # Incoming data is the transformation matrix of the haptic device end effector plus start and end bytes
            data = self.sock.recv(512)
            # print(data)
            stringdata = data.decode('utf-8')
            numdata = stringdata.split(";")
            numdata = numdata[0:-1] # Remove empty entry at end due to split
            # print(numdata)
            # numdata has an empty character appended to it as the last element
            # if (len(numdata) > 3): # Need a better filter here - start and end bits?
            # print(len(numdata))
            if ('S' in numdata):
                startIndex = numdata.index('S')
                # print("start index: ", startIndex)
                if ((startIndex == 0) & (len(numdata) == 18)):
                    self.omniX = float(numdata[13])
                    self.omniY = float(numdata[14])
                    self.omniZ = float(numdata[15])
                    print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
                    # matrix is the transformation matrix of device tip, listed by columns
                    matrix = numdata[1:17]
                    # convert to floats
                    fltMatrix = [float(i) for i in matrix]
                    print(fltMatrix)

                    self.tMatrix = np.array([[fltMatrix[0], fltMatrix[4], fltMatrix[8], fltMatrix[12]],\
                                            [fltMatrix[1], fltMatrix[5], fltMatrix[9], fltMatrix[13]],\
                                            [fltMatrix[2], fltMatrix[6], fltMatrix[10], fltMatrix[14]],\
                                            [fltMatrix[3], fltMatrix[7], fltMatrix[11], fltMatrix[15]]])
                    print(self.tMatrix)

            # count = 1
            # for i in numdata:
            #     if i == "S":
            #         if len(numdata)-count >= 4:
            #             if numdata[count+3] == "E": #index is count minus one (then plus x for relevant digit), for 0 indexing
            #                 self.omniX = float(numdata[count])
            #                 self.omniY = float(numdata[count+1])
            #                 self.omniZ = float(numdata[count+2])
            #                 print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
            #                 break
            #             else:
            #                 # print(numdata[count+3])
            #                 break # just ignore and wait for nex one
            #                 # raise ValueError('Somehow the endByte is not what was expected') # Somehow the endByte isn't what was expected
            #     else:
            #         count+=1

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
        # print("Omni coords: ", self.omniX, self.omniY, self.omniZ)
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


if __name__ == "__main__":
    phntmOmni = omniStreamer()
    omni_connected = phntmOmni.connectOmni()
    print("Haptic device connected? ", omni_connected)
    count = 0
    limit = 30
    while (count < limit):
        phntmOmni.getOmniCoords()
        [xMap, yMap, zMap] = phntmOmni.omniMap()
        print(xMap, yMap, zMap)
        time.sleep(0.5)
        count += 1