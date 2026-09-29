# Use pyUSB to communicate with a dualshock 4 controller

import usb.core
import usb.util
import usb.backend.libusb1
import time
import threading
import numpy as np
import math as mt
import platform

# See https://www.psdevwiki.com/ps4/DS4-USB for details on data indices


# For generic controller in xbox input mode: Device: device.idVendor=1118, device.idProduct=654
# VENDOR_ID = 0x045e
# PRODUCT_ID = 0x028e
VENDOR_ID_GULI = 0x057e
PRODUCT_ID_GULI = 0x2009

# DS4 controller IDs
VENDOR_ID_PS4 = 0x54c # = 1356 in decimal
PRODUCT_ID_PS4 = 0x9cc #0x5c4 = 1476 in decimal #0x9cc = 2508 in decimal


if platform.system() == "Windows":
	# Fol old asus laptop:
	# BACKEND = usb.backend.libusb1.get_backend(find_library=lambda x: "C:\\Users\\msrun\\Documents\\InflatableRobotControl\\ControlSystemThree\\.venv-deploy\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")
	# For new asus laptop:
	BACKEND = usb.backend.libusb1.get_backend(find_library=lambda x: "C:\\Users\\msrun\\Documents\\Inflatable Robot Control\\ControlSystem3\\venv-deploy\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")
	#"C:\\Users\\msrun\\Documents\\Inflatable Robot Control\\ControlSystem3\\venv-deploy\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")
else:
	# Don't forget to change the path to libusb-1.0.dll
	BACKEND = usb.backend.libusb1.get_backend() 







devices = usb.core.find(find_all=True, backend=usb.backend.libusb1.get_backend())
for device in devices:
	# print(f"Device: {device.idVendor=}, {device.idProduct=}")
	if VENDOR_ID_GULI == device.idVendor:
		print("Gulikit controller detected")
		VENDOR_ID = VENDOR_ID_GULI
		PRODUCT_ID = PRODUCT_ID_GULI
		INTERFACE_DS4 = 0
		SETTING_DS4 = 0
		ENDPOINT_DS4_OUT = 0

	elif VENDOR_ID_PS4 == device.idVendor:
		print("PS4 controller detected")
		VENDOR_ID = VENDOR_ID_PS4
		PRODUCT_ID = PRODUCT_ID_PS4
		INTERFACE_DS4 = 3 # 0 # HID interface number in cfg list
		SETTING_DS4 = 0
		ENDPOINT_DS4_OUT = 0 # Input endpoint



class ps4USB(threading.Thread):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.name = "ps4Thread"
		self.alive = True
		self._connection_made = threading.Event()
		self._lock = threading.Lock()
		self.paused = False

		self.controller = None

		self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=BACKEND)
		if VENDOR_ID == 0x54c:
			self.using_PS4 = True
		else:
			self.using_PS4 = False

		#self.dev.set_configuration()
		# print(self.dev)
		if self.dev is not None:
			self.cfg = self.dev.get_active_configuration()
			# print(self.cfg)
			self.interface = self.cfg[(INTERFACE_DS4, SETTING_DS4)]
			# print("Interface  ",  self.interface)
			# self.endpoint = self.interface[ENDPOINT_DS4_OUT]
			self.endpoint = usb.util.find_descriptor(
				self.interface,
				custom_match=lambda e:
				usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
				)

			if self.using_PS4:
				self.controller = True
				print("PS4 controller connected:", self.controller)

			else: # Using Gulikit in Switch mode, so need to initialise
				self.endpoint_out = usb.util.find_descriptor(
					self.interface,
					custom_match=lambda e:
					usb.util.endpoint_direction(e.bEndpointAddress)
					== usb.util.ENDPOINT_OUT
					)
				
				response = self.send_command(0x01)
				response = self.send_command(0x02)
				if response[0] == 0x30:
					self.controller = True
					print("Guli connected:", self.controller)
				else:
					self.send_command(0x03)
					self.send_command(0x02)
					self.send_command(0x04)
					self.data = self.endpoint.read(64, timeout=1000)
					if self.data[0] == 0x30:
						self.controller = True
						print("Guli connected:", self.controller)

			if platform.system() == "Linux":
				if self.dev.is_kernel_driver_active(0):
					self.dev.detach_kernel_driver(0)

				usb.util.claim_interface(self.dev, 0)	
			# try:
			# 	for i in range(interfaceNo+1):
			# 		# print(i)
			# 		if self.dev.is_kernel_driver_active(i):
			# 			self.dev.detach_kernel_driver(i)
			# except usb.core.USBError as e:
			# 	print(f"Could not detach kernel driver: {e}")

		else:
			self.controller = None

		self.data = None

		self.xChange = None
		self.yChange = None
		self.pChange = None

		self.axialChange = None
		self.rotChange = None
		self.wristChange = None
		self.toolChange = None
		self.graspChange = None

		self.thetaChange = None
		self.phiChange = None
		self.radChange = None

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
		self.XY_DEADTHRESH = 0.05
		self.PRISM_CHANGE = 1
		self.TOOL_CHANGE = 1
		self.GRASP_CHANGE = 1
		self.XY_SENSITIVITY = 0.2
		self.X_SENSITIVITY = 1.25
		self.Y_SENSITIVITY = 0.75
		self.PHI_SENSITIVITY = 0.5 #0.0087 approx half a degree
		self.THETA_SENSITIVITY = 0.5/2
		self.P_SENSITIVITY = 0.05
		self.TOOL_SENSITIVITY = 0.15
		self.GRASP_SENSITIVITY = 0.025


		###################################################################
		# Gantry robot geometry and limits
		###################################################################
		#Limits on axial extension
		self.MIN_EXTEND = 0.5 # mm
		self.MAX_EXTEND = 100 # mm
		
		# Limits on angle change of torque coil / rotary axis
		self.MIN_ROTARY = -400.0   # degrees 
		self.MAX_ROTARY =  400.0   # degrees 
		self.targDirRot = 1
		self.prevDirRot = 1
		self.antiHystDegRot = 360/360

		# Limits on wrist angle
		self.MIN_WRIST_ANGLE = 0   # degrees 
		self.MAX_WRIST_ANGLE = 120  # degrees 
		# Geometry of wrist motor
		# self.HYPOT_TIP = 5         # mm
		# self.TIP_CUTAWAY_WIDTH = 5 # mm
		# self.NUM_SUBSECTIONS = 5   # number of cutaways
		# self.RADIUS_SPOOL = 15     # mm
		# self.THETA_REST = 2*mt.asin((self.TIP_CUTAWAY_WIDTH/2)/self.HYPOT_TIP)

		# Limits on how far instrument can be extended
		self.MIN_TOOL_EXT = -1000      # mm
		self.MAX_TOOL_EXT = 1000    # mm
		# Geometry of tool extensionm motor 
		# self.TOOL_EXT_ROLLER_RADIUS = 5 #mm

		# Limits on grasper control
		self.MIN_GRASP_POS = 0   # mm 
		self.MAX_GRASP_POS = 2.5   # mm



	def stopped(self):
		return self._connection_made.is_set()



	def run(self):

		while True:
			if self.stopped():
				return
			if self.controller is not None:
				try:
					self.data = self.endpoint.read(0x40)
					if self.using_PS4:
						if self.decodePS4(self.data):
							self.getChanges()
					
					else:
						if self.decodeGulikit(self.data):
							self.getChanges()

				except Exception as e:
					print(f"Error with PS4 controller: {e}")
					self.controller = None
				
				
				
			else:
				# Ps4 controller not connected 
				# try to reconnect
				print("Trying reconnect")
				try:
					self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=BACKEND)
					if self.dev is not None:
						self.cfg = self.dev.get_active_configuration()
						self.interface = self.cfg[(INTERFACE_DS4, SETTING_DS4)]
						self.endpoint = usb.util.find_descriptor(
							self.interface,
							custom_match=lambda e:
							usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
							)

						if self.using_PS4:
							self.controller = True
						else: # Using Gulikit in Switch mode, so need to initialise
							self.endpoint_out = usb.util.find_descriptor(
								self.interface,
								custom_match=lambda e:
								usb.util.endpoint_direction(e.bEndpointAddress)
								== usb.util.ENDPOINT_OUT
								)

							#Initialise Gulikit controller
							self.send_command(0x01)
							self.send_command(0x02)
							self.send_command(0x03)
							self.send_command(0x02)
							response = self.send_command(0x04)
							print(response[0])
							if response[0] == 0x30:
								self.controller = True

				except Exception as e:
					# print(f"Error with PS4 controller: {e}")
					self.controller = None
			# print(self.RstickX , self.RstickY, self.R1, self.R2)



	def getChanges(self):
	# print("Data getter:", self.RstickH, self.RstickV)
		# Rotary
		if (abs(self.RstickX) > self.XY_DEADTHRESH):
			self.phiChange = float(self.PHI_SENSITIVITY*self.RstickX)
			self.xChange = -self.X_SENSITIVITY*self.RstickX
			# print("RstickH", self.RstickH)
			# print("Phi Change: ", self.phiChange)
		else:
			self.xChange = float(0)
			self.phiChange = float(0)


		# Wrist 
		if (abs(self.RstickY) > self.XY_DEADTHRESH):
			self.yChange = self.Y_SENSITIVITY*self.RstickY
			# self.thetaChange = self.THETA_SENSITIVITY*self.RstickY
			# print("Theta Change: ", self.phiChange)

		else:
			self.yChange = float(0)
			self.thetaChange = float(0)

		# normR2 = (self.R2 + self.TRIGGER_SHIFT)/self.TRIGGER_RANGE
		# print("normalised R2: ",normR2, self.R2)

		# Axial coord
		if (self.R1):
			self.pChange = -self.P_SENSITIVITY*self.PRISM_CHANGE
			self.radChange = -self.P_SENSITIVITY*self.PRISM_CHANGE
		elif (abs(self.R2) > self.R2_DEADTHRESH):
			self.pChange = self.P_SENSITIVITY*self.PRISM_CHANGE
			self.radChange = self.P_SENSITIVITY*self.PRISM_CHANGE
			# print("R2", self.R2)
		else:
			self.pChange = 0
			self.radChange = 0
		self.axialChange = self.pChange

		# Tool extension
		if (self.SquButton):
			self.toolChange = -self.TOOL_SENSITIVITY*self.TOOL_CHANGE
		elif (self.TriButton):
			self.toolChange = self.TOOL_SENSITIVITY*self.TOOL_CHANGE
		else:
			self.toolChange = 0

		# Grasper control increment
		if (self.CroButton):
			self.graspChange = self.GRASP_SENSITIVITY*self.GRASP_CHANGE
		elif (self.CirButton):
			self.graspChange = -self.GRASP_SENSITIVITY*self.GRASP_CHANGE
		else:
			self.graspChange = 0



	def getUserInputs(self):
		return self.xChange, self.yChange, self.pChange
	
	def incrementCylCoords(self, cAxialPos, cRotaryPos, cToolExt, cWristAngle, cGrasper):
		
		# Axial position
		if self.axialChange is not None:
			nAxialPos = cAxialPos + self.axialChange
		else:
			nAxialPos = cAxialPos

		if (nAxialPos < self.MIN_EXTEND):
			nAxialPos = self.MIN_EXTEND
		elif (nAxialPos > self.MAX_EXTEND):
			nAxialPos = self.MAX_EXTEND
		# print(nAxialPos)

		# Rotary position
		if self.xChange is not None:
			nRotaryPos = cRotaryPos + self.xChange
		else:
			nRotaryPos = cRotaryPos

		if (nRotaryPos < self.MIN_ROTARY):
			nRotaryPos = self.MIN_ROTARY
		elif (nRotaryPos > self.MAX_ROTARY):
			nRotaryPos = self.MAX_ROTARY

		# Tool extension
		if self.toolChange is not None:
			nToolExt = cToolExt + self.toolChange
		else:
			nToolExt = cToolExt

		if (nToolExt < self.MIN_TOOL_EXT):
			nToolExt = self.MIN_TOOL_EXT
		elif (nToolExt > self.MAX_TOOL_EXT):
			nToolExt = self.MAX_TOOL_EXT

		# Wrist position
		if self.yChange is not None:
			nWristAngle = cWristAngle + self.yChange
		else:
			nWristAngle = cWristAngle

		if (nWristAngle < self.MIN_WRIST_ANGLE):
			nWristAngle = self.MIN_WRIST_ANGLE
		elif (nWristAngle > self.MAX_WRIST_ANGLE):
			nWristAngle = self.MAX_WRIST_ANGLE

		# Grasp position
		if self.graspChange is not None:
			nGrasper = cGrasper + self.graspChange
		else:
			nGrasper = cGrasper

		nAxialPos = round(nAxialPos,2)
		nRotaryPos = round(nRotaryPos,2)
		nToolExt = round(nToolExt,2)
		nWristAngle = round(nWristAngle,2)
		nGrasper = round(nGrasper,2)

		return nAxialPos, nRotaryPos, nToolExt, nWristAngle, nGrasper
		


	def incrementXYZCoords(self, cX, cY, cZ, degreesToRotate,  LEVER_POINT = None):
		#TODO If motion limits reached (esp prismatic) do not change inputs - Don't let Z coord get too low or high 
		#TODO Encoder check on uSteppers blocking operation? - is sleep causing delay in messages to arduino? Observed pause before usteppers reset 
		#TODO Load cell calibration
		#TODO Check calibration routine
		#TODO New flags for ps4 controller initialisation (try to onnect if haptic not used, or if useOmni but connection failed)
		# print("Input coords: ", cX, cY, cZ)
		if self.xChange is None:
			self.xChange = 0

		if self.yChange is None:
			self.yChange = 0

		# Add rotation of coordinates after mapping
		changeMatrix = np.array([[self.xChange],\
								 [self.yChange]])
        
		if degreesToRotate is not None:
			radsToRotate = np.radians(degreesToRotate)
		else:
			radsToRotate = 0

		changeRotated = np.array([ [np.cos(radsToRotate), -np.sin(radsToRotate)],\
								   [np.sin(radsToRotate),  np.cos(radsToRotate)]])

		changeRotated = np.dot(changeRotated, changeMatrix)

		changeX = changeRotated[0]
		changeY = -changeRotated[1]

		if self.xChange is not None:
			# method .item() converts numpy to native python type
			nX = cX + changeX.item()
		else:
			nX = cX

		if self.yChange is not None:
			nY = cY + changeY.item()
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
	

	
	def incrementSphereCoords(self, c_theta, c_azimuth, c_prism, degreesToRotate):

		# cAltX = -cX - LEVER_POINT[0]
		# cAltY = cZ - LEVER_POINT[2]
		# cAltZ = cY - LEVER_POINT[1]

		degreesToRotate = 0

		if self.thetaChange is None:
			self.thetaChange = float(0)

		if self.phiChange is None:
			self.phiChange = float(0)

		# Add rotation of coordinates after mapping
		changeMatrix = np.array([[self.thetaChange],\
								 [self.phiChange]])
        
		if degreesToRotate is not None:
			radsToRotate = np.radians(degreesToRotate)
		else:
			radsToRotate = 0

		changeRotated = np.array([ [np.cos(radsToRotate), -np.sin(radsToRotate)],\
								   [np.sin(radsToRotate),  np.cos(radsToRotate)]])

		changeRotated = np.dot(changeRotated, changeMatrix)


		# print("X Stick: ", self.RstickX)
		# print("Y Stick: ", self.RstickY)
		# print("X Change: ", self.xChange)
		# print("Y Change: ", self.yChange)
		# print("Phi Change: ", self.phiChange)
		# print("Theta Change: ", self.thetaChange)
		changeTheta = changeRotated[0]
		changePhi = -changeRotated[1]
		# print(changeTheta.item())
		# print(changePhi.item())

		# Calculate spherical coordinates:
		cTheta = c_theta #mt.atan2(mt.sqrt(cAltX**2 + cAltY**2), cAltZ) 
		cPhi = c_azimuth # mt.atan2(cAltY, cAltX) 
		cPrism = c_prism #mt.sqrt((cAltX)**2 + (cAltY)**2 + (cAltZ)**2)

		# Increment the theta, phi and radius as input from controller:

		if self.thetaChange is not None:
			# method .item() converts numpy to native python type
			nTheta = cTheta + changeTheta.item()*mt.pi/180
		else:
			nTheta = cTheta

		if self.phiChange is not None:
			nPhi = cPhi + changePhi.item()*mt.pi/180
		else:
			nPhi = cPhi

		if self.radChange != 0:
			nPrism = cPrism + self.radChange
		else:
			nPrism = cPrism

		# # Convert back to shperical coordinates
		# nAltX = nRadius*mt.sin(nTheta)*mt.cos(nPhi)
		# nAltY = nRadius*mt.sin(nTheta)*mt.sin(nPhi)
		# nAltZ = nRadius*mt.cos(nTheta)

		# #Do the inverse
		# nX = -(nAltX + LEVER_POINT[0])
		# nY = nAltZ + LEVER_POINT[1]
		# nZ = nAltY + LEVER_POINT[2]

		# print("Change in coords:")
		# print(cX - nX)
		# print(cY - nY)
		# print(cZ - nZ)
		# print()

		# nX = round(nX,2)
		# nY = round(nY,2)
		# nZ = round(nZ,2)
		return nTheta, nPhi, nPrism



	def getPSButtonData(self):
        # 0 for no buttons, 1 for dark grey/cross (open), 2 for light grey/circle (close) button, 3 for both
		self.ps4Buttons = 0
		if self.CroButton:
			self.ps4Buttons = 1
		elif self.CirButton:
			self.ps4Buttons = 2
		elif self.SquButton:
			self.ps4Buttons = 3
		elif self.TriButton:
			self.ps4Buttons = 4
		return self.ps4Buttons
	


	def stop_ps4(self):
		self.disconnectController()
		self._connection_made.set()
		try:
			self.join(timeout = 1)
		except RuntimeError as re:
			print(re)
		finally:
			print("Is ps4 thread still alive? ", self.is_alive())


	def send_command(self, command):

		packet = bytearray(64)
		packet[0] = 0x80
		packet[1] = command
		# print(f"\nSEND: 80 {command:02X}")
		self.endpoint_out.write(packet, timeout=1000)

		try:
			response = self.endpoint.read(64, timeout=1000)
			# print("RECV:"," ".join(f"{x:02X}" for x in response))
			return response

		except usb.core.USBTimeoutError:
			print("No response")
			return None
		

	def decodePS4(self, data):
		# print(self.data)
		self.RstickX = (data[3] - 2**7)/2**7
		self.RstickY = (data[4] - 2**7)/2**7
		# print("Stick axes: ", self.RstickX,  self.RstickY)

		self.SquButton = data[5] & 2**4  !=0
		self.CroButton = data[5] & 2**5  !=0
		self.CirButton = data[5] & 2**6  !=0
		self.TriButton = data[5] & 2**7  !=0

		self.R1 = data[6] & 2**1  != 0
		self.R2 = data[9]/2**8

		return True


	def decodeGulikit(self, data):
		if data[0] != 0x30:
			return False

		# -------------------------
		# Right analogue stick
		# -------------------------

		rx = data[9] | ((data[10] & 0x0F) << 8)
		ry = (data[10] >> 4) | (data[11] << 4)

		self.RstickX = (rx - 2048) / 2048
		self.RstickY = -((ry - 2048) / 2048)
		# print(self.RstickX, self.RstickY)

		# -------------------------
		# Buttons
		# -------------------------
		buttons = data[3]

		self.CroButton = (buttons & 0x04) != 0
		self.CirButton = (buttons & 0x08) != 0
		self.SquButton = (buttons & 0x01) != 0
		self.TriButton = (buttons & 0x02) != 0

		self.R1 = (buttons & 0x40) != 0
		self.R2 = 1.0 if (buttons & 0x80) else 0.0

		return True
		

	def disconnectController(self):
		if self.dev is not None:
			try:
				usb.util.release_interface(self.dev, 0)
				if platform.system() == "Linux":
					self.dev.attach_kernel_driver(0)
			except usb.core.USBError:
				pass




if __name__ == "__main__":
	ps4 = ps4USB()
	# print(ps4.controller)
	if ps4.controller is not None:
		ps4.start()

	cX, cY, cZ = 0, 0, 0
	num = 500
	rotateDegrees = 0

	axialPos, rotaryPos, toolExt, wristAngle, graspPos = 0,0,0,0,0
	[desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos] = ps4.incrementCylCoords(axialPos, rotaryPos, toolExt, wristAngle, graspPos)

	while num > 0:
		# ps4.updateAndMapPS4()
		ps4Buttons = ps4.getPSButtonData()
		controllerButtons = ps4Buttons
		# print(controllerButtons)    
		# [xPS4, yPS4, zPS4] = ps4.incrementXYZCoords(cX, cY, cZ, rotateDegrees)
		# cX, cY, cZ = xPS4, yPS4, zPS4


		[desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos] = ps4.incrementCylCoords(axialPos, rotaryPos, toolExt, wristAngle, graspPos)
		axialPos, rotaryPos, toolExt, wristAngle, graspPos = desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos
		print(desAxialPos, desRotaryPos, desTooExt, desWristAngle, desGraspPos)

		num -= 1

		time.sleep(0.1)

	if ps4.controller is not None:
		ps4.stop_ps4()



