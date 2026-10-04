
import time
import csv
import os
import platform



class ardLogger():

    def __init__(self):
        self.ardData = []
        self.tempData = []
        self.numRows = 0

        if platform.system() == "Windows":
            self.parent = "C:/Users/msrun/Documents/Inflatable Robot Control/ControlSystem3/control/"
        else:
            self.parent = "/home/lannsair/Documents/DataLogs/DT Prime/"

        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "logs/pumps/arduinoLogs " + self.logTime + ".csv"
        self.fileName = os.path.join(self.parent, self.relative) # USE THIS IN REAL TESTS
        # self.fileName = 'ardLogFile.csv' # For test purposes
        with open(self.fileName, mode ='w', newline='') as arduinoLog1: 
            ardLog1 = csv.writer(arduinoLog1)
            ardLog1.writerow(['Axial', 'Rotary', 'Tool', 'Wrist', 'Grasp',
                              'Sensor1', 'Sensor2', 'Sensor3', 'Sensor4', 'Sensor5',
                              'Load1', 'Load2','Load3', 'Load4',
                              'ArduinoTime',
                              'StartTime: ', time.time()])


    def ardLogAll(self, stepList, pressList, timeL, loadList):
        [realStepL, realStepR, realStepT, realStepP] = stepList
        [pressL, pressR, pressT, pressP, regulatorSensor] = pressList
        [loadL, loadR, loadT, loadP] = loadList
        self.tempData.extend([realStepL] + [realStepR] + [realStepT] + [realStepP] +
                             [pressL] + [pressR] + [pressT] + [pressP] + [regulatorSensor] +
                             [loadL] + [loadR] + [loadT] + [loadP] +
                             timeL)
        self.ardData.append(self.tempData)
        self.tempData = []
        return


    def ardSave(self):
        """
        Save ardLog list into csv file
        """
        with open(self.fileName, 'a', newline='') as arduinoLog2:
            ardLog2 = csv.writer(arduinoLog2)
            for i in range(len(self.ardData)):
                ardLog2.writerow(self.ardData[i])
        return