import serial
import serial.tools.list_ports


# serInfo = serial.tools.list_ports.comports()
# for i in serInfo:
#     print(i) 
import time
import csv
import os
import numpy as np

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/pose/pose " + logTime + ".csv"
fileName = os.path.join(parent, relative)
with open(fileName, mode ='w', newline='') as poseLog1: 
    logger1 = csv.writer(poseLog1)
    logger1.writerow(['X_est', 'Y_est', 'Z_est', 'Alpha', 'Beta', 'Gamma', 'MS Classification', 'Timestamp', time.time()])


miniScanDirectory = "logs/pose/mini "+ logTime
miniScanPath = os.path.join(parent, miniScanDirectory)
os.mkdir(miniScanPath)
# rasterNumber = 0
# miniScanRelative = "logs/pose/mini "+ logTime +"/raster" + str(rasterNumber) + ".csv"
# miniScanFileName = os.path.join(parent, miniScanRelative)


def unpack_homo(homo):
    R = homo[:,:3][:3]
    t = homo[:,-1][:3]
    return R,t

class massSpec:
# Make a serial connection to mass spec computer
    def __init__(self):
        self.msSerial = serial.Serial()
        self.poseData = []
        self.msClass = 0
        self.doAblationAlgorithm = True
        self.miniRaster = []

        self.rasterNumber = 0
        self.miniScanRelative = miniScanDirectory + "/raster" + str(self.rasterNumber) + ".csv"
        self.miniScanFileName = os.path.join(parent, self.miniScanRelative)



    # Connect
    def connect(self, COMport):
        # connected = False
        # remainingCOM = str(set(COMlist) - set(pumpSer)).strip("{}'")
        # if remainingCOM != 'set()':
        try:
            self.msSerial.port = COMport
            self.msSerial.baudrate = 9600
            self.msSerial.timeout = 0
            self.msSerial.open()
            self.msSerial.reset_input_buffer()
            connected = True
        except serial.SerialException:
            connected = False
            return connected
        return connected
        

    def sendState(self, state):
        if state == "Stop":
            stateIndicator = 0
        elif state == "Run":
            stateIndicator = 1
        elif state == "Last":
            stateIndicator = 2
        elif state == "STOP":
            stateIndicator = 9

        # Encode and send to fibrebot DAQ
        message = str(stateIndicator) + "\n"
        message = message.encode('utf-8')
        self.msSerial.write(message)


    def receiveState(self):
        if self.msSerial.in_waiting > 0:
            reply = self.msSerial.readline().strip()
            self.msSerial.reset_input_buffer()
            result = reply.decode('ascii')
            # print(reply)

            # Check received message for start and end bytes "<>"
            # process result to extract classification, timestamp
        
            if result == "B":
                #Fibrebot motion complete
                self.msClass = result
                return result
            else:
                return None


    def logMiniScan(self, T_Rob_Inst_Est, rotVect):
        if self.msClass is None:
            msClassification = ' '
        else:
            msClassification = self.msClass

        if type(T_Rob_Inst_Est) is list:
            self.miniRaster.append([T_Rob_Inst_Est[0]] + [T_Rob_Inst_Est[1]] + [T_Rob_Inst_Est[2]] + [float(rotVect[0])] + [float(rotVect[1])] + [float(rotVect[2])] + [msClassification] + [time.time()])
        elif T_Rob_Inst_Est is None:
            self.miniRaster.append([' '] + [' '] + [' '] + [' '] + [' '] + [' '] + [msClassification] + [time.time()])
        else:
            # [R, T] = unpack_homo(T_Rob_Inst_Est)
            T = [T_Rob_Inst_Est[0,3], T_Rob_Inst_Est[1,3], T_Rob_Inst_Est[2,3]]
            # realX = T_Rob_Inst_Est[0,3]
            # realY = T_Rob_Inst_Est[1,3]
            # realZ = T_Rob_Inst_Est[2,3]
            # print("Position", -realZ + 15, realY + 8.66, realX)
            # R9 = R.reshape(1,9)

            self.miniRaster.append([T[0]] + [T[1]] + [T[2]] + [float(rotVect[0])] + [float(rotVect[1])] + [float(rotVect[2])] + [msClassification] + [time.time()])


    def saveMiniScan(self):
        with open(self.miniScanFileName, 'a', newline='') as posLog3:
            positionLog3 = csv.writer(posLog3)
            for i in range(len(self.miniRaster)):
                positionLog3.writerow(self.miniRaster[i])
        #Prepare for next raster scan:
        self.rasterNumber += 1
        self.miniScanRelative = "logs/pose/mini"+ logTime +"/raster" + str(self.rasterNumber) + ".csv"
        self.miniScanFileName = os.path.join(parent, self.miniScanRelative)
        self.miniRaster = []
        


    def logPose(self, T_Rob_Inst_Est, rotVect):
        if self.msClass is None:
            msClassification = ' '
        else:
            msClassification = self.msClass

        if T_Rob_Inst_Est is None:
            self.poseData.append([' '] + [' '] + [' '] + [' '] + [' '] + [' '] + [msClassification] + [time.time()])
        else:
            # [R, T] = unpack_homo(T_Rob_Inst_Est)
            T = [T_Rob_Inst_Est[0,3], T_Rob_Inst_Est[1,3], T_Rob_Inst_Est[2,3]]
            # realX = T_Rob_Inst_Est[0,3]
            # realY = T_Rob_Inst_Est[1,3]
            # realZ = T_Rob_Inst_Est[2,3]
            # print("Position", -realZ + 15, realY + 8.66, realX)
            # R9 = R.reshape(1,9)

            self.poseData.append([T[0]] + [T[1]] + [T[2]] + [float(rotVect[0])] + [float(rotVect[1])] + [float(rotVect[2])] + [msClassification] + [time.time()])

    def savePoseMassSpec(self):
        with open(fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])