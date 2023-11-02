import socket
import time
import subprocess
import numpy as np
import os
import math
# from ctypes import *
np.set_printoptions(suppress=True, precision = 2)

# location = os.path.dirname(__file__)
# parent = os.path.dirname(location)
# relative = "modules\Transformation_And_Forces.exe"
# fileName = os.path.join(parent, relative).replace('\\', '/') # For subprocess it looks like we need forward slashes in path
# fileName = 'C:/Users/msrun/Documents/InflatableRobotControl/ControlSystemThree/control/modules/HelloHapticDevice.exe'
MAX_FORCE = 2 # N

class omniStreamer():
    def __init__(self):
        self.server_addr = ('localhost', 8888)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.omniX = 0.0 # mm
        self.omniY = 0.0
        self.omniZ = 0.0
        self.tMatrix = []
        self.omniServer = None
        self.omniButton = 0 # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both
        self.manualTimeoutCounter = 0

        self.location = os.path.dirname(__file__)
        self.parent = os.path.dirname(self.location)
        self.relative = "modules\Transformation_And_Forces.exe"
        self.fileName = os.path.join(self.parent, self.relative).replace('\\', '/') # For subprocess it looks like we need forward slashes in path

    def connectOmni(self, noSubProcess):
        # problem with this was that it waited for program to terminate, which never happens, but stdin=None, stdin=subprocess.DEVNULL, stdout=None, stderr=None argumetns sorted this
        # print(fileName)
        if not noSubProcess:
            self.omniServer = subprocess.run(self.fileName,\
                check = True, capture_output = False, stdin = subprocess.DEVNULL, stdout = None, stderr = None)
        try:
            if not noSubProcess:
                self.sock.connect(self.server_addr)
            # self.sock.setblocking(0)
            self.sock.settimeout(0.01)
            # print("Connected to {:s}".format(repr(self.server_addr)))
            # print(self.sock)
            return self.checkConnection()
        except AttributeError as ae:
            # print("Error creating the socket: {}".format(ae))
            return False
        except OSError as os:
            # print("Error creating socket: {}".format(os))
            return False


    def checkConnection(self):
        while (self.manualTimeoutCounter < 250):
            omniDataReceived = self.getOmniCoords()
            if omniDataReceived: return True
        return False

# [-0.500,-0.500,00.000]

# [0.500,0.500,0.000]
# 0
# 1 - 6
# 8 - 13
# 15 - 20
# 21
    def getOmniCoords(self, forces = None):
        try:
            handshake = b'1'
            if forces is not None:
                forcesStrList = []# [format(x, '.3f') for x in forces]
                # print(forcesStrList)
                for x in range(len(forces)):
                    if forces[x] >= 0:
                        if forces[x] > MAX_FORCE:
                            forces[x] = MAX_FORCE
                        forcesStrList.append('0' + format(abs(forces[x]), '1.3f'))

                    else:
                        if (abs(forces[x]) > MAX_FORCE):
                            forces[x] = -1*MAX_FORCE
                        forcesStrList.append(format(forces[x], '1.3f'))

                forcesString = '[' + forcesStrList[0] + ',' + forcesStrList[1] + ',' + forcesStrList[2] + ']' 
                handshake = forcesString.encode('utf-8')
                # print(handshake)
            self.sock.send(handshake)

            # Incoming data is the transformation matrix of the haptic device end effector plus start and end bytes
            data = self.sock.recv(512)
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
                if ((startIndex == 0) & (len(numdata) == 19)):
                    self.omniX = float(numdata[13])
                    self.omniY = float(numdata[14])
                    self.omniZ = float(numdata[15])
                    self.omniButton = int(numdata[17])
                    # print(self.omniButton)
                    # print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
                    
                    # matrix is the transformation matrix of device tip, listed by columns
                    matrix = numdata[1:17]
                    # convert to floats
                    fltMatrix = [float(i) for i in matrix]
                    # print(fltMatrix)

                    self.tMatrix = np.array([[fltMatrix[0], fltMatrix[4], fltMatrix[8], fltMatrix[12]],\
                                            [fltMatrix[1], fltMatrix[5], fltMatrix[9], fltMatrix[13]],\
                                            [fltMatrix[2], fltMatrix[6], fltMatrix[10], fltMatrix[14]],\
                                            [fltMatrix[3], fltMatrix[7], fltMatrix[11], fltMatrix[15]]])

                    self.rotMatrix = np.array([[fltMatrix[0], fltMatrix[4], fltMatrix[8]],\
                                            [fltMatrix[1], fltMatrix[5], fltMatrix[9]],\
                                            [fltMatrix[2], fltMatrix[6], fltMatrix[10]]])
                    
                    self.manualTimeoutCounter = 0
                    return 1

        except socket.timeout:
            self.manualTimeoutCounter += 1
            if self.manualTimeoutCounter > 250:
                print("Check connection to Geomagic Touch / Phantom Omni")
                return 0
        except socket.error as se:
            print("Exception on socket: {}".format(se))
            # print("Closing socket")
            self.sock.close()
            return 2




    def omniMap(self, degreesToRotate = None):#, xFromOmni, yFromOmni, zFromOmni):
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


        #Calibrated position in inkwell
        homeX = 0.0
        homeY = -65.51071
        homeZ = -88.11420

        # 160 W x 120 H x 70 D mm    From datasheet, but real values are different
        rangeXOmni = 430
        rangeYOmni = 315
        rangeZOmni = 220

        minX = -215
        maxX = 215
        minY = -112
        maxY = 203
        minZ = -123
        maxZ = 95

        offsetX = (minX + maxX)/2
        offsetY = (minY + maxY)/2 # -45.5
        offsetZ = (minZ + maxZ) 

        # print(self.omniX, self.omniY, self.omniZ)

        # Fixing coords from Omni to go from -0.5*range to 0.5 range
        # x is -ve to the left and +ve to the right  horizontal
        xUnit = (self.omniX - offsetX)/(rangeXOmni)
        # y is -ve down and +ve up                   vertical
        yUnit = (self.omniY - offsetY)/(rangeYOmni)
        # z is -ve towards body, +ve towards user    depth
        zUnit = (self.omniZ - offsetZ)/(rangeZOmni) - 0.2
        # print(xUnit, yUnit, zUnit)

        sensX = 1.5
        sensY = 1.5
        sensZ = 1.5

        signX = -1
        signY = 1
        signZ = -1

        rangeXWorkspace = 62.5
        rangeYWorkspace = 55
        rangeZWorkspace = 50

        centreWorkspaceX = 0
        centreWorkspaceY = 2.6263
        centreWorkspaceZ = 43.25


        # x is +ve to the left and -ve to the right  horizontal
        xMapped = signX * sensX * xUnit * rangeXWorkspace + centreWorkspaceX
        # y is -ve down and +ve up                   vertical
        yMapped = signY * sensY * yUnit * rangeYWorkspace + centreWorkspaceY
        # z is +ve towards body, -ve towards user    depth
        zMapped = signZ * sensZ * zUnit * rangeZWorkspace + centreWorkspaceZ
        # print(xMapped, yMapped, zMapped)
        # print()

        #TODO Add rotation of coordinates after mapping
        mapMatrix = np.array([  [xMapped],\
                                [yMapped],\
                                [zMapped]])
        
        if degreesToRotate is not None:
            radsToRotate = np.radians(degreesToRotate)
        else:
            radsToRotate = 0

        omniRotate = np.array([ [np.cos(radsToRotate), -np.sin(radsToRotate), 0],\
                                [np.sin(radsToRotate),  np.cos(radsToRotate), 0],\
                                [0,                     0,                    1]])
        
        coordsMappedRotated = np.dot(omniRotate, mapMatrix)

        xMappedRotated = float(coordsMappedRotated[0])
        yMappedRotated = float(coordsMappedRotated[1])
        zMappedRotated = float(coordsMappedRotated[2])



        # xMapped = -1*(self.omniX*((46-(-46))/440)) + 9.455 # abs just for test       approx 0.2093
        # yMapped = 1*(self.omniY*((55-(-35))/310))  + 5.4591 #                        approx 0.2903
        # zMapped = -1*(self.omniZ*((75-30)/215))*2 + 10 # Omni direction is opposite to real direction     approx 0.2091
        # if (xMapped < 0):
        #     xMapped = 0
        # if (yMapped < 0):
        #     yMapped = 0
        # if (zMapped < 0):
        #     zMapped = 0
        # if (zMapped > 38.5):
        #     zMapped = 38.5
        # print("x: ", xMapped, ", y: ", yMapped, ", z: ", zMapped)
        return xMappedRotated, yMappedRotated, zMappedRotated
        
        

    def omniClose(self):
        self.sock.close()


if __name__ == "__main__":
    phntmOmni = omniStreamer()

    omni_connected = phntmOmni.connectOmni(0)
    print("Haptic device connected? ", omni_connected)
    count = 0
    limit = 400
    forces = [0, 0, 0]
    while (count < limit):

        # Set forces to send to device
        # forces = [2*math.sin(0.1*count), 2*math.cos(0.1*count), 2*math.cos(0.1*count)]
        # Both send forces and receive pose information
        # print(phntmOmni.omniButton)
        omniDataReceived = phntmOmni.getOmniCoords(forces)

        # if omniDataReceived == 2: break
        [xMap, yMap, zMap] = phntmOmni.omniMap()
        # print(xMap, yMap, zMap)
        # print(phntmOmni.tMatrix)
        # print(phntmOmni.omniServer.stdout)
        time.sleep(0.5)
        count += 1
        # print(math.sin(count))
    phntmOmni.omniClose()