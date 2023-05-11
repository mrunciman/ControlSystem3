import serial
import serial.tools.list_ports
import numpy as np


# serInfo = serial.tools.list_ports.comports()
# for i in serInfo:
#     print(i) 

# FIBRE_NAME = "ASRL1::INSTR"


startByte = "<"
endByte = ">"
completeMessage = '999'

class fibreBot:
# Make a serial connection to the NI controller of the fibrebot
    def __init__(self):
        self.fibreSerial = serial.Serial()
        self.fibreLength = 80
        self.rasterSize = 5
        self.miniScanDone = False
        self.lineAngle = 0

    # Connect
    # def connect(self, pumpSer, COMlist):
    def connect(self):
        connected = False
        # remainingCOM = str(set(COMlist) - set(pumpSer)).strip("{}'")
        # if remainingCOM != 'set()':
        remainingCOM = 'COM4'
        try:
            self.fibreSerial.port = remainingCOM
            self.fibreSerial.baudrate = 9600
            self.fibreSerial.timeout = 0
            self.fibreSerial.open()
            self.fibreSerial.reset_input_buffer()
            connected = True
        except serial.SerialException:
            connected = False
            return connected
        return connected
        

    def sendState(self, state):
        if state == "Stop":
            stateIndicator = "S"
        elif state == "Run":
            stateIndicator = "T"
        elif state == "Raster":
            stateIndicator = "R" + str(self.rasterSize)
        # elif state == "STOP":
        #     stateIndicator = 9
        else:
            # stateIndicator = "L" + str(float(self.angle)) # pass through the direction of motion
            stateIndicator = "L" + "{angle:.3f}".format(angle = self.lineAngle) # pass through the direction of motion
        # print(stateIndicator)

        # Encode and send to fibrebot DAQ
        message = str(stateIndicator) + "\n"
        message = message.encode('utf-8')
        self.fibreSerial.write(message)

    def receiveState(self, isConnected):
        # This flag will trigger analysis/imaging of mini raster data:
        self.miniScanDone = False
        if isConnected:
            if self.fibreSerial.in_waiting > 0:
                #Receive forward kinematic model of fibre
                #  - tip position received will be XY 
                reply = self.fibreSerial.readline().strip()
                # self.fibreSerial.reset_output_buffer()
                reply = reply.decode('ascii')
                self.fibreSerial.reset_input_buffer()
                startOK = False
                endOK = False
                startIndex = 0
                endIndex = 0

                # if reply.startswith(startByte):
                #     startOK = True

                if startByte in reply:
                    startOK = True
                    startIndex = reply.index(startByte)

                
                # if reply.endswith(">"):
                #     endOK = True

                if endByte in reply:
                    endOK = True
                    endIndex = reply.index(endByte)
                
                # stripped = reply.strip('<>')

                # fibreXY = stripped.split(',')
                # print(reply)
                # if reply == "B":
                #     #Fibrebot motion complete
                #     return True
                signX = 0
                signY = 0
                if (startOK and endOK) and (startIndex < endIndex):
                    fibreXY = reply[startIndex+1:endIndex].split(',')
                # if startOK and endOK:
                    # try:
                    filtX = filter(str.isdigit, fibreXY[0])
                    filtY = filter(str.isdigit, fibreXY[1])
                    fibreX = "".join(filtX) # TODO interpret as float and multiply with sign
                    fibreY = "".join(filtY)
                    if fibreX == "":
                        fibreX = 0
                    else:
                        fibreX = float(fibreXY[0])

                    if fibreY == "":
                        fibreY = 0
                    else:
                        fibreY = float(fibreXY[1])
                    # except:
                        # if fibreXY[0].startswith("-"): signX = -1
                        # if fibreXY[1].startswith("-"): signY = -1
                        # filtX = filter(str.isdigit, fibreXY[0])
                        # filtY = filter(str.isdigit, fibreXY[1])
                        # fibreX = "".join(filtX) # TODO interpret as float and multiply with sign
                        # fibreY = "".join(filtY)

                    if fibreXY[0] == completeMessage:
                        if fibreXY[1] == completeMessage:
                            self.miniScanDone = True
                    # print(fibreX)
                    # print(fibreY)
                else:
                    fibreX = 0
                    fibreY = 0
            else:
                fibreX = 0
                fibreY = 0
        else:
            fibreX = 0
            fibreY = 0

        fibreZ = self.fibreLength

        T_tip_fibre = np.block([[1, 0, 0, fibreX],\
                                [0, 1, 0, fibreY],\
                                [0, 0, 1, fibreZ],\
                                [0, 0, 0, 1]])
            
        return T_tip_fibre

        

