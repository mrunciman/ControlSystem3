import serial
import serial.tools.list_ports



print(serial.tools.list_ports.ListPortInfo) 

FIBRE_NAME = "ASRL1::INSTR"

# Use COM port whose hwid matches the DAQ
# serial.tools.list_ports.ListPortInfo


class fibreBot:


    def __init__(self):
        self.fibreSerial = serial.Serial()

    def connect(self, pumpSer, COMlist):
        remainingCOM = set(COMlist) - set(pumpSer)

        self.fibreSerial.port = remainingCOM
        self.fibreSerial.baudrate = 9600
        self.fibreSerial.timeout = 0
        self.fibreSerial.open()
        self.fibreSerial.reset_input_buffer()

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

            if reply == "B":
                #Fibrebot motion complete
                return 1
            else:
                return 0

        

