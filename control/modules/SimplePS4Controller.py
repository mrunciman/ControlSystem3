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

import os
import pygame
import threading
import time
from clsControllerData import ControllerData




class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.
                
                # os.system('cls')
                # print(self.button_data)
                # print(self.axis_data)
                # print(self.hat_data)



class SimplePS4Controller(threading.Thread):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stopper = threading.Event()
        self.cTime = time.ctime
        self.paused = False
        """Initialize the joystick components"""
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()        

    def stop_ps4(self):
        self._stopper.set()
        try:
            self.join()
            print("Is ps4 thread still alive? ", self.is_alive())
        except RuntimeError as re:
            print(re)


    def stopped(self):
        return self._stopper.isSet()


    def pause(self):
        self.paused = True
        #this is should make the calling thread wait if pause() is
        #called while the thread is 'doing the thing', until it is
        #finished 'doing the thing'

    #should just resume the thread
    def resume(self):
        self.paused = False


    def run(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
              
            if self.stopped():
                return
            if not self.paused:
                for event in pygame.event.get():
                    #print "Changed!"
                    ControllerData.changed=True
                    ControllerData.newData=True
                    if event.type == pygame.JOYAXISMOTION:
                        self.axis_data[event.axis] = round(event.value,2)

                    elif event.type == pygame.JOYBUTTONDOWN:
                        self.button_data[event.button] = True
                    elif event.type == pygame.JOYBUTTONUP:
                        self.button_data[event.button] = False
                        
                    elif event.type == pygame.JOYHATMOTION:
                        #print (event.value)
                        self.hat_data[event.hat] = event.value

                    # Insert your code on what you would like to happen for each event here!
                    # In the current setup, I have the state simply printing out to the screen.
                    
                    

                    #print("buttuns Data")
                    #print(self.button_data)
                    #print("Axis Data")
                    print(self.axis_data)
                    ControllerData.button_data=self.button_data
                    ControllerData.axis_data = self.axis_data
                    ControllerData.simplfyData()
                else:
                    ControllerData.changed =False

            self.cTime= time.ctime()
            #print(self.cTime)







if __name__ == "__main__":
    # ps4 = PS4Controller()
    # ps4.init()
    # ps4.listen()

    ps4 = SimplePS4Controller() # Create an object from controller
    ps4.start() #start and listen to events



    i=100
    while(i>0):
       
        time.sleep(1)
        # os.system('cls')
        if ControllerData.newData == True: # hmmm , I got a new data from the controller 
            # print("Button Data")
            # print(ControllerData.button_data)         
            print(ControllerData.axis_data)
            # x = ControllerData.axis_data[0]

            # y = (ControllerData.axis_data[1]*-1) # flip the y data 
            # angle =  ControllerData.getAngle360(0,0,ControllerData.L_Ball_H,ControllerData.L_Ball_V)
            # print("Angle : "+str(angle))
            # print ("Simplified Data")
            # ControllerData.printSimplifiedValues()
            # Set the New Data as Old data ( I got it , and used it )
            ControllerData.newData=False
        # print(ps4.cTime)
        # print ("timer"+str(i))
        i = i-1

    ps4.stop_ps4() #stop listening