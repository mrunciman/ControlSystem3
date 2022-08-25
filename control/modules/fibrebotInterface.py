import serial
import serial.tools.list_ports


# serInfo = serial.tools.list_ports.comports()
# for i in serInfo:
#     print(i) 

# FIBRE_NAME = "ASRL1::INSTR"

class fibreBot:
# Make a serial connection to the NI controller of the fibrebot
    def __init__(self):
        self.fibreSerial = serial.Serial()

    # Connect
    def connect(self, pumpSer, COMlist):
        connected = False
        remainingCOM = str(set(COMlist) - set(pumpSer)).strip("{}'")
        if remainingCOM != 'set()':
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
        self.fibreSerial.write(message)

    def receiveState(self):
        if self.fibreSerial.in_waiting > 0:
            reply = self.fibreSerial.readline().strip()
            self.fibreSerial.reset_input_buffer()
            reply = reply.decode('ascii')
            print(reply)
            if reply == "B":
                #Fibrebot motion complete
                return True
            else:
                return False

        

