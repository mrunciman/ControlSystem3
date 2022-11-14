import serial
import serial.tools.list_ports


# serInfo = serial.tools.list_ports.comports()
# for i in serInfo:
#     print(i) 


class massSpec:
# Make a serial connection to mass spec computer
    def __init__(self):
        self.msSerial = serial.Serial()

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
                return result
            else:
                return None