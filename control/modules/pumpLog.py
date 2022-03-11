
import time
import csv
import os

# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
# location = "Imperial College London/Imperial/Fluidic Control/ControlSystem/logs/pumps"
location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/pumps/arduinoLogs " + logTime + ".csv"
fileName = os.path.join(parent, relative) # USE THIS IN REAL TESTS
# fileName = 'ardLogFile.csv' # For test purposes
with open(fileName, mode ='w', newline='') as arduinoLog1: 
    ardLog1 = csv.writer(arduinoLog1)
    ardLog1.writerow(['S_LHS', 'Lc_LHS', 'A_LHS', 'M_LHS', 'P_LHS', 'P_LMed', 'T_LHS',\
        'S_RHS', 'Lc_RHS', 'A_RHS', 'M_RHS', 'P_RHS', 'P_RMed', 'T_RHS',\
        'S_TOP', 'Lc_TOP','A_TOP', 'M_TOP', 'P_TOP', 'P_TMed', 'T_TOP',\
        'S_PRI', 'Lc_PRI','A_PRI', 'M_PRI', 'P_PRI', 'P_PMed', 'T_PRI',\
        'C_LHS', 'C_RHS', 'C_TOP', 'C_DEG', time.time()])



class ardLogger():

    def __init__(self):
        self.ardData = []
        self.tempData = []
        self.numRows = 0

    def ardLog(self, secondstep, length, theta, primarystep, press, median, timems):
        """
        Save stepCount, master cable lengths, pressure values and time 
        from pumps in a list to later save in csv.
        """
        # self.ardData[self.numRows] = self.ardData[self.numRows] + [secondstep] + [length] + [theta] + [primarystep] + [press] + [median] + [timems]
        self.tempData.extend([secondstep] + [length] + [theta] + [primarystep] + [press] + [median] + [timems])
        return

    def ardLogCollide(self, lhsC, rhsC, topC, degC):
        # self.ardData[self.numRows] = self.ardData[self.numRows] + [lhsC] + [rhsC] + [topC] + [degC]
        self.tempData.extend([lhsC] + [rhsC] + [topC] + [degC])
        self.ardData.append(self.tempData)
        self.tempData = []
        return


    def ardSave(self):
        """
        Save ardLog list into csv file
        """
        with open(fileName, 'a', newline='') as arduinoLog2:
            ardLog2 = csv.writer(arduinoLog2)
            for i in range(len(self.ardData)):
                ardLog2.writerow(self.ardData[i])
        return