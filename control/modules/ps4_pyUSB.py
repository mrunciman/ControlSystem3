# Use pyUSB to communicate with a dualshock 4 controller

import usb.core
import usb.util
import usb.backend.libusb1
import sys
import os
import time

# See https://www.psdevwiki.com/ps4/DS4-USB for details on data indices

# DS4 controller ids (might be different on your side)
VENDOR_ID = 0x54c
PRODUCT_ID = 0x5c4 #0x9cc

# Don't forget to change the path to libusb-1.0.dll
BACKEND = usb.backend.libusb1.get_backend(find_library=lambda x: "C:\\Users\\msrun\\Documents\\Inflatable Robot Control\\ControlSystem3\\venv-deploy\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")

INTERFACE_DS4 = 0 # HID interface
SETTING_DS4 = 0

ENDPOINT_DS4_OUT = 0 # Input endpoint


class ps4USB():
	def __init__(self):
		self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=BACKEND)
		if self.dev is not None:
			self.cfg = self.dev.get_active_configuration()
			# print(cfg)
			self.interface = self.cfg[(INTERFACE_DS4, SETTING_DS4)]
			self.endpoint = self.interface[ENDPOINT_DS4_OUT]
			self.controller = self.endpoint.read(0x40)[0]
		else:
			self.controller = None
		print(self.endpoint.read(0x40)[0])

		self.data = None

		self.xChange = None
		self.yChange = None
		self.pChange = None

		self.ps4Buttons = 0 # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both

		self.R1 = False
		self.R2 = 0
		self.RstickX = 0
		self.RstickY = 0
		self.SquButton = False
		self.CroButton = False
		self.CirButton = False
		self.TriButton = False
		
		self.R2_DEADTHRESH = 0.25
		self.TRIGGER_RANGE = 2
		self.TRIGGER_SHIFT = 1
		self.XY_DEADTHRESH = 0.1
		self.PRISM_CHANGE = 0.1
		self.XY_SENSITIVITY = 10
		self.P_SENSITIVITY = 1



	def updateAndMapPS4(self):
		if self.controller is not None:
			self.data = self.endpoint.read(0x40)
			# print(self.data)

			self.RstickX = (self.data[3] - 2**7)/2**7
			self.RstickY = (self.data[4] - 2**7)/2**7

			self.SquButton = self.data[5] & 2**4  !=0
			self.CroButton = self.data[5] & 2**5  !=0
			self.CirButton = self.data[5] & 2**6  !=0
			self.TriButton = self.data[5] & 2**7  !=0

			self.R1 = self.data[6] & 2**1  != 0
			self.R2 = self.data[9]/2**8
			self.getChanges()

			# print(self.RstickX , self.RstickY, self.R1, self.R2)



	def getChanges(self):
	# print("Data getter:", self.RstickH, self.RstickV)
		if (abs(self.RstickX) > self.XY_DEADTHRESH):
			self.xChange = self.XY_SENSITIVITY*self.RstickX
			# print("RstickH", self.RstickH)
		else:
			self.xChange = 0

		if (abs(self.RstickY) > self.XY_DEADTHRESH):
			self.yChange = self.XY_SENSITIVITY*self.RstickY
		else:
			self.yChange = 0

		# normR2 = (self.R2 + self.TRIGGER_SHIFT)/self.TRIGGER_RANGE
		# print("normalised R2: ",normR2, self.R2)

		if (self.R1):
			self.pChange = -self.P_SENSITIVITY*self.PRISM_CHANGE
		elif (abs(self.R2) > self.R2_DEADTHRESH):
			self.pChange = self.P_SENSITIVITY*self.PRISM_CHANGE
			# print("R2", self.R2)
		else:
			self.pChange = 0



	def incrementXYZCoords(self, cX, cY, cZ):
		if self.xChange is not None:
			nX = cX + self.xChange
		else:
			nX = cX

		if self.yChange is not None:
			nY = cY + self.yChange
		else:
			nY = cY

		if self.pChange is not None:
			nZ = cZ + self.pChange
		else:
			nZ = cZ

		nX = round(nX,2)
		nY = round(nY,2)
		nZ = round(nZ,2)
		return nX, nY, nZ



	def getPSButtonData(self):
        # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both
		self.ps4Buttons = 0
		if self.CroButton:
			self.ps4Buttons = 1
		elif self.CirButton:
			self.ps4Buttons = 2
		return self.ps4Buttons



if __name__ == "__main__":
	ps4 = ps4USB()
	cX, cY, cZ = 0, 0, 0
	num = 100

	while num > 0:
		ps4.updateAndMapPS4()
		ps4Buttons = ps4.getPSButtonData()
		controllerButtons = ps4Buttons
		print(controllerButtons)    
		[xPS4, yPS4, zPS4] = ps4.incrementXYZCoords(cX, cY, cZ)
		cX, cY, cZ = xPS4, yPS4, zPS4
		print(xPS4, yPS4, zPS4)

		num -= 1
		time.sleep(0.1)



# trigger = 0

# while trigger != 255: 
	
# 	os.system('cls')

# 	# Axes
# 	print('Readout L stick  X:', format(endpoint.read(0x40)[1],'#04X'), format(endpoint.read(0x40)[1],'08b'), endpoint.read(0x40)[1])
# 	print('Readout L stick  Y:', format(endpoint.read(0x40)[2],'#04X'), format(endpoint.read(0x40)[2],'08b'), endpoint.read(0x40)[2])
# 	print('Readout R stick  X:', format(endpoint.read(0x40)[3],'#04X'), format(endpoint.read(0x40)[3],'08b'), endpoint.read(0x40)[3])
# 	print('Readout R stick  Y:', format(endpoint.read(0x40)[4],'#04X'), format(endpoint.read(0x40)[4],'08b'), endpoint.read(0x40)[4])
# 	print('Readout L2 trigger:', format(endpoint.read(0x40)[8],'#04X'), format(endpoint.read(0x40)[8],'08b'), endpoint.read(0x40)[8])
# 	print('Readout R2 trigger:', format(endpoint.read(0x40)[9],'#04X'), format(endpoint.read(0x40)[9],'08b'), endpoint.read(0x40)[9],'\n')

# 	# Accelerometers (2 bytes to signed integer)
# 	data = endpoint.read(0x40)
# 	print('Accelerometer X:', format(256*data[18]+data[19],'016b'), 256*data[18]+data[19]-(65536 if data[18] > 127 else 0))
# 	print('Accelerometer Y:', format(256*data[16]+data[17],'016b'), 256*data[16]+data[17]-(65536 if data[16] > 127 else 0))
# 	print('Accelerometer Z:', format(256*data[14]+data[15],'016b'), 256*data[14]+data[15]-(65536 if data[14] > 127 else 0),'\n')

# 	# Gyroscopes  (2 bytes to signed integer)
# 	data = endpoint.read(0x40)
# 	print('Gyroscope X (Roll) :', format(256*data[20]+data[21],'016b'), 256*data[20]+data[21]-(65536 if data[20] > 127 else 0))
# 	print('Gyroscope Y (Yaw)  :', format(256*data[22]+data[23],'016b'), 256*data[22]+data[23]-(65536 if data[22] > 127 else 0))
# 	print('Gyroscope Z (Pitch):', format(256*data[24]+data[25],'016b'), 256*data[24]+data[25]-(65536 if data[24] > 127 else 0),'\n')

# 	# Touch PAD (for more details please refer to the listed references)
# 	print('Touch PAD:', format(endpoint.read(0x40)[40],'#04X'), format(endpoint.read(0x40)[40],'08b'), endpoint.read(0x40)[40],'\n')

# 	# Hats
# 	print("Hats:", endpoint.read(0x40)[5]&15,'\n') #0,1,2,3,4,5,6,7,8

# 	# Buttons
# 	print("Square:", endpoint.read(0x40)[5]&16!=0)
# 	print("Cross:", endpoint.read(0x40)[5]&32!=0)
# 	print("Circle:", endpoint.read(0x40)[5]&64!=0)
# 	print("Triangle:", endpoint.read(0x40)[5]&128!=0,'\n')

# 	# Timestamp
# 	print('Timestamp:', endpoint.read(0x40)[7]>>2)

# 	# Battery status
# 	print('Battery:', endpoint.read(0x40)[30]%16) # 11 means it's Max and charging
# 	time.sleep(0.1)
	
# 	trigger = endpoint.read(0x40)[8] # press L2 fully to break
