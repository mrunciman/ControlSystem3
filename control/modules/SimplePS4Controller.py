#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# simple Playstation 4 Controller
#
# Copyright © TareqGamal ArabicRobotics 
# ArabicRobotics.com
# https://github.com/ArabicRobotics
# https://www.youtube.com/channel/UCj3IoLXlUfjYTHpwgXm4xsg
# https://www.codeproject.com/Members/Tareq-Gamal
# you can Add your code In File (Demo.py) No Change in this file


import pygame
import threading
import time

if __name__ == "__main__":
    from cProfile import Profile
    from pstats import SortKey, Stats 
# try:
#     from modules.clsControllerData import ControllerData
# except ModuleNotFoundError as e:
#     from clsControllerData import ControllerData


class SimplePS4Controller(threading.Thread):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    # axis_data = None
    button_data = None
    hat_data = None

    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = "ps4Thread"
        self.alive = True
        self._connection_made = threading.Event()
        self._lock = threading.Lock()
        self.paused = False

        """Initialize the joystick components"""
        self._lock.acquire()
        try:
            pygame.init()
            pygame.joystick.init()
            try:
                self.controller = pygame.joystick.Joystick(0)
                self.controller.init()
            except pygame.error as e:
                print(e)
        finally:
            self._lock.release()

        self.axis_data = {0:0, 1:0, 2:0, 3:0, 4:-1, 5:-1, 6:0, 7:0}
        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)


        self.xStick = None
        self.yStick = None
        self.pStick = None

        self.ps4Buttons = 0 # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both

        self.R1 = False
        self.R2 = 0
        self.RstickH = 0
        self.RstickV = 0
        self.Xbutton = False
        self.Obutton = False

        self.R2_DEADTHRESH = 0.25
        self.TRIGGER_RANGE = 2
        self.TRIGGER_SHIFT = 1
        self.XY_DEADTHRESH = 0.1
        self.PRISM_CHANGE = 0.1
        self.XY_SENSITIVITY = 10
        self.P_SENSITIVITY = 1

                

    def stop_ps4(self):
        self._connection_made.set()
        try:
            self.join()
            print("Is ps4 thread still alive? ", self.is_alive())
        except RuntimeError as re:
            print(re)


    def stopped(self):
        return self._connection_made.isSet()


    def pause(self):
        self.paused = True
        #this is should make the calling thread wait if pause() is called 

    #should just resume the thread
    def resume(self):
        self.paused = False


    def run(self):
        """Listen for events to happen"""
        
        while True:
              
            if self.stopped():
                return
            if not self.paused:
                
                self._lock.acquire()
                try:
                    for event in pygame.event.get():
                        # print(event)

                        if event.type == pygame.JOYAXISMOTION:
                            self.axis_data[event.axis] = round(event.value,2)

                        elif event.type == pygame.JOYBUTTONDOWN:
                            self.button_data[event.button] = True
                        elif event.type == pygame.JOYBUTTONUP:
                            self.button_data[event.button] = False
                            
                        elif event.type == pygame.JOYHATMOTION:
                            self.hat_data[event.hat] = event.value

                    
                        # print("Buttons", self.button_data)
                        # print("Axes", self.axis_data)
                            
                        self.R1 = self.button_data[10]
                        self.R2 = self.axis_data[5]
                        self.RstickH = self.axis_data[2]
                        self.RstickV = self.axis_data[3]
                        self.Xbutton = self.button_data[0]
                        self.Obutton = self.button_data[1]
                finally:
                    self._lock.release()
    


    def getStickData(self):
        # print("Data getter:", self.RstickH, self.RstickV)
        if (abs(self.RstickH) > self.XY_DEADTHRESH):
            self.xStick = self.XY_SENSITIVITY*self.RstickH
            # print("RstickH", self.RstickH)
        else:
            self.xStick = 0

        if (abs(self.RstickV) > self.XY_DEADTHRESH):
            self.yStick = self.XY_SENSITIVITY*self.RstickV
        else:
            self.yStick = 0


        normR2 = (self.R2 + self.TRIGGER_SHIFT)/self.TRIGGER_RANGE
        # print("normalised R2: ",normR2, self.R2)

        if (self.R1):
            self.pStick = -self.P_SENSITIVITY*self.PRISM_CHANGE
        elif (abs(normR2) > self.R2_DEADTHRESH):
            self.pStick = self.P_SENSITIVITY*self.PRISM_CHANGE
            # print("R2", self.R2)
        else:
            self.pStick = 0

    def incrementXYZCoords(self, cX, cY, cZ):
        if self.xStick is not None:
            nX = cX + self.xStick
        else:
            nX = cX

        if self.yStick is not None:
            nY = cY + self.yStick
        else:
            nY = cY

        if self.pStick is not None:
            nZ = cZ + self.pStick
        else:
            nZ = cZ

        nX = round(nX,2)
        nY = round(nY,2)
        nZ = round(nZ,2)
        return nX, nY, nZ
    

    def getPSButtonData(self):
        # 0 for no buttons, 1 for dark grey (far), 2 for light grey (close) button, 3 for both
        self.ps4Buttons = 0
        if self.Xbutton:
            self.ps4Buttons = 1
        elif self.Obutton:
            self.ps4Buttons = 2
        return self.ps4Buttons



def test():
    cX, cY, cZ = 0, 0, 0

    ps4 = SimplePS4Controller() # Create an object from controller
    if ps4.controller is not None:
        ps4.start() #start and listen to events
        print("Connected to ps4 controller")

        numLoops = 100
        loops = numLoops
        seconds = 2
        while(loops>0):
        
            time.sleep(seconds/numLoops)
            # print("Sticks:", ps4.xStick, ps4.yStick, ps4.pStick)
            print("Coords\t:", cX, "\t", cY, "\t", cZ)

            ps4.getStickData()
            ps4Buttons = ps4.getPSButtonData()
            [xPS4, yPS4, zPS4] = ps4.incrementXYZCoords(cX, cY, cZ)
            cX, cY, cZ = xPS4, yPS4, zPS4

            loops = loops-1

        ps4.stop_ps4() #stop listening
        print(xPS4, yPS4, zPS4)
        return xPS4, yPS4, zPS4
    





if __name__ == "__main__":
    
    with Profile() as profile:
    
        print(f"{test()}")
        (
            Stats(profile)
            .strip_dirs()
            .sort_stats(SortKey.CALLS)
            .print_stats()
        )