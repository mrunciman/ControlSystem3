"""
Set of functions to facilitate connection to syringe pumps, which are controlled by arduinos.
Arduinos connected to a USB hub so COM port names must be checked in main code.
Enables connection, pressure monitoring, and step count monitoring for later burst
protection and volume calculation.
"""
import serial 
import serial.tools.list_ports
import time
import numpy as np

# def ardConnect():
#     # serial.tools.list_ports.comports()
#     comlist = []
#     for comport in serial.tools.list_ports.comports():
#         comlist.append(comport.device)
#     print(comlist)

#     PUMPNAMES = ['LHS', 'RHS']#, 'TOP', 'PRI']#, 'PNEU']
#     pumpDict = {}
#     serialDict = {}

#     reply = b""
#     replyASCII = ""
#     replyFlag = 0
#     MAXREPLYNO  = len(PUMPNAMES)
#     MESSAGE = "M\n".encode('utf-8')
#     # MESSAGE = MESSAGE.encode('utf-8')

#     # Find the pumps and determine their names, otherwise flag that they're not connected
    
#     # Open each device in COM port list
#     for device in comlist:
#         replyFlag = 0
#         with serial.Serial() as ser:
#             ser.baudrate = 115200
#             ser.port = device
#             ser.timeout = 0
#             ser.open()
#             # print(ser)
#             ser.reset_input_buffer()
#             #Leave time to initialise
#             time.sleep(1)

#             while(replyFlag != MAXREPLYNO):
#                 ser.write(MESSAGE)
#                 #delay before reading reply
#                 time.sleep(1)
#                 x = 'e'
#                 reply = b""
#                 while ord(x) != ord("\n"):
#                     x = ser.read()
#                     if x == b"":
#                         break
#                     if x == b"\n":
#                         break
#                     reply = reply + x
#                 # reply = ser.readline()
#                 # reply = reply.strip()
#                 ser.reset_input_buffer()
#                 print("Reply: ", reply)
#                 replyASCII = reply.decode('ascii')
#                 print("ReplyASCII: ", replyASCII)

#                 if replyASCII in PUMPNAMES:
#                     index = PUMPNAMES.index(replyASCII) # Which one of pumpNames was reply
#                     pumpDict.update({PUMPNAMES[index]:device})
#                     serialDict.update({device:ser})
#                     replyASCII = ""
#                     break
#                 replyFlag += 1
#                 # ser.reset_input_buffer()
#         # print(pumpDict)

#     return pumpDict, serialDict, PUMPNAMES, comlist

class ardInterfacer:

    def __init__(self, namePump, ser):
        self.PUMP_NAME = namePump
        # self.portNumber = numPort
        # self.ser = serial.Serial()
        # self.ser.port = 'COM%s' % (self.portNumber) 
        # self.ser.baudrate = 115200
        # self.ser.timeout = 0
        self.ser = ser
        self.stepCountBU = 0
        self.pumpTimeBU = 0
        self.stepMessage = "M\n"
        self.stepMessEnc = self.stepMessage.encode('utf-8')

        self.press1 = 0.0
        self.press2 = 0.0
        self.press3 = 0.0
        self.press4 = 0.0
        self.press5 = 0.0
        self.press6 = 0.0
        self.press7 = 0.0
        self.press8 = 0.0
        self.press9 = 0.0
        self.press10 = 0.0
        self.pressArray = np.array([self.press1, self.press2, self.press3, self.press4, self.press5, \
            self.press6, self.press7, self.press8, self.press9, self.press10])
        self.pressMed = np.median(self.pressArray)
        self.pressMedPrev = self.pressMed

        self.derivThresh = 200
        self.deriv2Thresh = 300

        self.conDetected = False
    

    # def connect(self):
    #     """
    #     Each pump requires verification of the cable it is contolling (top, lhs, or rhs).
    #     Pumps will function normally when the pumpName sent here matches the name hardcoded 
    #     on each arduino.
    #     e.g. top = connect("TOP", 4)
    #     """
    #     # self.ser.open()
    #     connected = False
    #     message = self.PUMP_NAME + "\n"
    #     # reply = ""
    #     message = message.encode('utf-8')    #Encode message
    #     print(message)
    #     time.sleep(1)     #give arduino time to set up (there are delays in arduino code for pressure sensor)
    #     attemptNo = 0
    #     while(attemptNo <= 20):
    #         self.ser.write(message)
    #         time.sleep(0.5)     #delay before sending message again
    #         # self.ser.write(message)
    #         # print("Handshake: ", message)
    #         if self.ser.in_waiting > 0:
    #             x = 'e'
    #             reply = b""
    #             while ord(x) != ord("\n"):
    #                 x = self.ser.read()
    #                 if x == b"":
    #                     # x = "e"
    #                     break
    #                 elif x == b"\n":
    #                     break
    #                 reply = reply + x
    #             reply = reply.strip()
    #             # self.ser.reset_input_buffer()
    #             print("Reply: ", reply)
    #             replyASCII = reply.decode('ascii')
    #             print("ReplyASCII: ", replyASCII)
    #             reply = reply.decode('ascii')

    #             if reply == self.PUMP_NAME:
    #                 connected = True
    #                 self.ser.reset_output_buffer()
    #                 return connected
    #         attemptNo += 1

    #     # return open serial connection to allow pumps to be controlled in main code
    #     return connected

    def connect(self): 
        """ 
        Each pump requires verification of the cable it is contolling (top, lhs, or rhs). 
        Pumps will function normally when the pumpName sent here matches the name hardcoded  
        on each arduino. 
        e.g. top = connect("TOP", 4) 
        """ 
        time.sleep(2)
        self.ser.open()
        connected = False 
        message = self.PUMP_NAME + "\n" 
        self.stepMessEnc = message.encode('utf-8') 
        reply = b"" 
        replyASCII = "" 
        message = message.encode('utf-8') #Encode message 
        time.sleep(2) #give arduino time to set up (there are delays in arduino code for pressure sensor) 
        while(replyASCII != self.PUMP_NAME): 
            x = "e" 
            reply = b"" 
            while ord(x) != ord("\n"): 
                x = self.ser.read() 
                if x == b"": 
                    self.ser.write(message) 
                    time.sleep(0.5)     #delay before sending message again 
                    x = "e"             # assign a rnadom value as x can't be empty 
                else: 
                    reply = reply + x 
            reply = reply.strip() 
            print("Reply: ", reply) 
            replyASCII = reply.decode('ascii') 
            print("ReplyASCII: ", replyASCII) 
            # reply = reply.decode('ascii') 
        self.ser.write(message)  # one more time for luck
        time.sleep(0.5)
        connected = True 
        # self.ser.readline()
        # self.ser.reset_output_buffer()
        # return open serial connection to allow pumps to be controlled in main code 
        return connected 



    def sendStep(self, stepNumber):
        """
        This function sends ideal position (stepNumber) then receives
        the real step count (stepCount) from arduino.
        steps = sendStep(serialConnection, stepNumber)
        """
        if type(stepNumber) != str:
            stepString = "{:04d}".format(stepNumber)
        else:
            stepString = stepNumber
        message = "S" + stepString + "s" + "\n"
        # print("Message: ", repr(message))
        # self.ser.reset_output_buffer()
        self.stepMessEnc = message.encode('utf-8')
        # print("Message: ", self.stepMessEnc)
        self.ser.write(self.stepMessEnc)
        return



    def listenReply(self):
        x = "e"
        stepPress = b""
        reply = b""

        # while self.ser.in_waiting == 0:
        #     continue
        # time.sleep(0.005)
        # reply = self.ser.readline()
        # stepPress = reply



        while ord(x) != ord("\n"):
            x = self.ser.read()
            if x == b"":
                x = "e"
            elif x == b"E":
                break
            else:
                stepPress = stepPress + x

        # print("Message: ", self.stepMessEnc)
        # print(stepPress)
        stepPress = stepPress.decode('utf-8')
        stepPress = stepPress.strip("E\r\n")
        # print(stepPress)
        stepPress = stepPress.split(',')
        # print(stepPress)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # IF STEP COUNT = L_'pumpName' STOP AND DISCONNECT ALL
        # print(stepPress)
        if len(stepPress) == 3:
            stepCount = stepPress[0]
            self.stepCountBU = stepCount

            filtPump = filter(str.isdigit, stepPress[1])
            pumpPress = "".join(filtPump)
            if pumpPress == "":
                pumpPress = 0
            pumpPress = int(pumpPress)*10 # pressure in Pascals
            # pumpPress = int(stepPress[1])/10 # pressure in mbar
            # pumpPress = int(stepPress[1])

            filtTime = filter(str.isdigit, stepPress[2])
            pumpTime = "".join(filtTime)
            if pumpTime == "":
                pumpTime = 0
            pumpTime = int(pumpTime)
            self.pumpTimeBU = pumpTime
        elif stepPress == ['']:
            # print("First value is empty")
            stepCount = self.stepCountBU
            pumpPress = self.pressMed
            pumpTime = self.pumpTimeBU
        else:
            stepCount = self.stepCountBU
            pumpPress = self.pressMed
            pumpTime = self.pumpTimeBU
            # print("Arduino output not three long")

        return stepCount, pumpPress, pumpTime


    def listenZero(self, isPumpZero, pressIn, timeIn):
        # If calibration done, don't wait for arduino
        if (isPumpZero == True):
            stepCount = '000000' + str(self.PUMP_NAME)
            pumpPress = pressIn
            pumpTime = timeIn
            return stepCount, pumpPress, pumpTime
        else:
            [stepCount, pumpPress, pumpTime] = self.listenReply()
            return stepCount, pumpPress, pumpTime
 

    def newPressMed(self, newPress):
        self.press10 = self.press9
        self.press9 = self.press8
        self.press8 = self.press7
        self.press7 = self.press6
        self.press6 = self.press5
        self.press5 = self.press4
        self.press4 = self.press3
        self.press3 = self.press2
        self.press2 = self.press1
        self.press1 = newPress
        self.pressArray = np.array([self.press1, self.press2, self.press3, self.press4, self.press5,\
            self.press6, self.press7, self.press8, self.press9, self.press10])
        self.pressMedPrev = self.pressMed
        self.pressMed = np.median(self.pressArray)

        return self.pressMed

    def derivPress(self, newTime, prevTime):
        realTimeStep = (newTime - prevTime)/1000
        # Prevent div by zero errors / limit noise?
        if realTimeStep <= 0.001:
            realTimeStep = 0.001
        pressDiff = self.pressMed - self.pressMedPrev
        deriv = pressDiff/realTimeStep
        deriv2 = deriv/realTimeStep
        # Check derivatives against heuristically determined threshold values to determine contact:
        self.conDetected = (abs(deriv) > self.derivThresh) & (abs(deriv2) > self.deriv2Thresh)
        # 1 means actuator in tension, -1 means compression, 0 means no contact
        self.conDetected = self.conDetected*np.sign(deriv)
        return self.conDetected, deriv

    def initPress(self, pressIndex, pressVal):
        self.pressArray[pressIndex] = float(pressVal)


