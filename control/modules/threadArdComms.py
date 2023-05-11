import threading
import serial 
import serial.tools.list_ports
from serial.threaded import LineReader, Protocol, Packetizer
import time


startByte = "<"
endByte = ">"
CORRECT_LENGTH = 13


class ardThreader:

    def __init__(self):
        self.ser = serial.Serial()
        self.ser.port = 'COM10'
        self.ser.baudrate = 115200
        self.ser.timeout = 1
        try:
            self.ser.open()
            self.ser.reset_input_buffer()
            self.connected = True
        except serial.SerialException as e:
            self.connected = False
            print(e)


        #LocalReaderThread is the transport, SerialReaderProtocolLine is the protocol
        self.t = LocalReaderThread(self.ser, SerialReaderProtocolLine)
        print(self.t)

        self.positionData = [0.0, 0.0, 0.0, 0.0]
        self.pressData = [0, 0, 0, 0, 0]
        self.ardTime = 0

    def startThreader(self):
        self.t.start()
        transport, self.protocol = self.t.connect()

 
    def sendStep(self, stepNumber, controlState = None, inflationState = None):
        """
        This function sends ideal position (stepNumber) then receives
        the real step count (stepCount) from arduino.
        steps = sendStep(serialConnection, stepNumber)
        """
        if type(stepNumber) != str:
            stepString = "{:06d}".format(stepNumber)
        else:
            stepString = stepNumber
        # If we are sending the extra state variables, alter the 
        # output message appropriately
        if controlState is not None:
            msg = self.setState(controlState, inflationState)
            message = msg + stepString + ',' + stepString + ',' + stepString + ',' + stepString + ',' + stepString + "\n"
        else:
            message = "S" + stepString + "\n"
        print("Message: ", repr(message))
        # message = message.encode('utf-8')
        self.protocol.write_line(message)
        return
    

    def getData(self):
        self.positionData = self.t.positionD
        self.pressData = self.t.pressD
        self.ardTime = self.t.ardT
        print(self.positionData)
        print(self.pressData)
        print(self.ardTime)
    


    def setState(self, controlState, inflationState):
        # Choice of control states of C, H, or S
        # calibration state is C
        # Hold state is H
        # Active mode is S
        if controlState == 1:
            msg = "C"
        elif controlState == 2:
            msg = "H"
        elif controlState == 3:
            msg = "S"
        # Choice of inflation states of I or D
        # inflated state is I
        # deflated is D
        if inflationState == 0:
            msg = msg + "I"
        elif inflationState == 1:
            msg = msg + "D"
        else:
            msg = msg + "_" # Decimal 95
        return msg






class LocalReaderThread(threading.Thread):
    """\
    Implement a serial port read loop and dispatch to a Protocol instance (like
    the asyncio.Protocol) but do it with threads.

    Calls to close() will close the serial port but it is also possible to just
    stop() this thread and continue the serial port instance otherwise.
    """

    def __init__(self, serial_instance, protocol_factory):
        """\
        Initialize thread.

        Note that the serial_instance' timeout is set to one second!
        Other settings are not changed.
        """
        # If a subclass overrides the constructor, it must make sure to invoke
        # the base class constructor (Thread.__init__()) before doing anything
        # else to the thread.
        super(LocalReaderThread, self).__init__()
        self.daemon = True
        self.serial = serial_instance
        self.protocol_factory = protocol_factory
        self.alive = True
        self._lock = threading.Lock()
        self._connection_made = threading.Event()
        self.protocol = None
        self.calibrationFlag = 'N'

        self.positionD = [0.0, 0.0, 0.0, 0.0]
        self.pressD = [0, 0, 0, 0, 0]
        self.ardT = 0

    def stop(self):
        """Stop the reader thread"""
        self.alive = False
        if hasattr(self.serial, 'cancel_read'):
            self.serial.cancel_read()
        self.join(2)

    # Overriding Thread's run method in this subclass
    def run(self):
        """Reader loop"""
        if not hasattr(self.serial, 'cancel_read'):
            self.serial.timeout = 1
        self.protocol = self.protocol_factory()
        try:
            self.protocol.connection_made(self)
        except Exception as e:
            self.alive = False
            self.protocol.connection_lost(e)
            self._connection_made.set()
            return
        error = None
        self._connection_made.set()
        while self.alive and self.serial.is_open:
            try:
                # read all that is there or wait for one byte (blocking)
                data = self.serial.read(self.serial.in_waiting or 1)
                print(data)
                data = b'<,1.23, 2.34, 3.45, 4.56, -00075.9, -48.4, -31.4, -21.3, -88.0,13001,N,>\n'
            except serial.SerialException as e:
                # probably some I/O problem such as disconnected USB serial
                # adapters -> exit
                error = e
                break
            else:
                if data:
                    # make a separated try-except for called user code
                    try:
                        self.protocol.data_received(data)
                        # This function is in the Packetizer, which then
                        # goes to the handle_packet function in LineReader,  both in __init__.py in serial.theaded
                        # Then handle_line is called from the local Protocol that inherits from LineReader
                    except Exception as e:
                        error = e
                        break
                else:
                    print("empty")
        self.alive = False
        self.protocol.connection_lost(error)
        self.protocol = None

    def write(self, data):
        """Thread safe writing (uses lock)"""
        with self._lock:
            return self.serial.write(data)

    def close(self):
        """Close the serial port and exit reader thread (uses lock)"""
        # use the lock to let other threads finish writing
        with self._lock:
            # first stop reading, so that closing can be done on idle port
            self.stop()
            self.serial.close()

    def connect(self):
        """
        Wait until connection is set up and return the transport and protocol
        instances.
        """
        if self.alive:
            self._connection_made.wait()
            if not self.alive:
                raise RuntimeError('connection_lost already called')
            return (self, self.protocol)
        else:
            raise RuntimeError('already stopped')

    # - -  context manager, returns protocol

    def __enter__(self):
        """
        Enter context handler. May raise RuntimeError in case the connection
        could not be created.
        """
        self.start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')
        return self.protocol

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Leave context: close port"""
        self.close()



class SerialReaderProtocolLine(LineReader):
    TERMINATOR = b'\n'
    ENCODING = 'utf-8'

    def connection_made(self, transport):
        """Called when reader thread is started"""
        super(SerialReaderProtocolLine, self).connection_made(transport)
        print("Connected, ready to receive data...")

    
    def handle_line(self, line):
        """New line waiting to be processed"""
        # print(line)
        #Example of "line" string: '0.00,-00075,     0.00,-48,     0.00,-31,     0.00,-21,-88,13001,E'
        #Future example of "line" string: '0.00, 0.00, 0.00, 0.00, -00075, -48, -31, -21, -88,13001,E'
        startOK = False
        endOK = False
        startIndex = 0
        endIndex = 0

        # Check that start and end bytes are present, in the correct order,
        # that start and end are at at start and end of string, respectively
        if startByte in line:
            startIndex = line.index(startByte)
            # print(startIndex)
            if startIndex == 0:
                startOK = True

        if endByte in line:
            endIndex = line.index(endByte)
            # print(endIndex)
            # print(len(line)-1)
            if (endIndex == len(line)-1):
                endOK = True

        if ((startOK & endOK) & (startIndex < endIndex)):
            stepPress = line.split(',')
            # print(stepPress)
            # print(len(stepPress))
            if len(stepPress)!= CORRECT_LENGTH:
                return
            
            # ['<', '1.23', ' 2.34', ' 3.45', ' 4.56', ' -00075', ' -48.4', ' -31.4', ' -21.3', ' -88.0', '13001', 'N', '>']

            # print(self.transport.positionD)
            # pos = [stepPress[0], stepPress[2], stepPress[4], stepPress[6]]
            # self.transport.positionD = [float(x) for x in pos]
            self.transport.positionD = [float(x) for x in stepPress[1:5]] # FOR FUTURE USE
            # print(self.transport.positionD)

            # press = [stepPress[1], stepPress[3], stepPress[5], stepPress[7], stepPress[8]]
            # self.transport.pressD = [int(y) for y in press]
            self.transport.pressD = [float(y) for y in stepPress[5:10]] # FOR FUTURE USE
            # print(self.transport.pressD)

            self.transport.ardT = int(stepPress[10])
            # print(self.transport.ardT)

            self.transport.calibrationFlag = stepPress[11]
            # print(self.transport.calibrationFlag)



    def connection_lost(self, exc):
        if exc:
            print('Port closed\n')






if __name__ == '__main__':

    ardThread = ardThreader()
    limit = 30
    count = 0
    if ardThread.connected:
        # ardThread.sendStep(34, 3, 0)
        print("Before while")
        ardThread.startThreader()
        print("After thread started")
        while(count < limit):
            time.sleep(0.05)
            count = count + 1
            # ardThread.sendStep(34, 3, 0)
            ardThread.getData()
            print("Here")

    ardThread.t.stop()
    ardThread.ser.close