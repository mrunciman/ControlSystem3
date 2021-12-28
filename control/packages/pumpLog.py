
import time
import csv
import os

# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
# location = "Imperial College London/Imperial/Fluidic Control/ControlSystem/logs/pumps"
location = os.path.dirname(__file__)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/pumps/arduinoLogs " + logTime + ".csv"
fileName = os.path.join(location, relative) # USE THIS IN REAL TESTS
# fileName = 'ardLogFile.csv' # For test purposes
with open(fileName, mode ='w', newline='') as arduinoLog1: 
    ardLog1 = csv.writer(arduinoLog1)
    ardLog1.writerow(['S_LHS', 'Lc_LHS', 'A_LHS', 'M_LHS', 'P_LHS', 'P_LMed', 'T_LHS',\
        'S_RHS', 'Lc_RHS', 'A_RHS', 'M_RHS', 'P_RHS', 'P_RMed', 'T_RHS',\
        'S_TOP', 'Lc_TOP','A_TOP', 'M_TOP', 'P_TOP', 'P_TMed', 'T_TOP',\
        'S_PRI', 'Lc_PRI','A_PRI', 'M_PRI', 'P_PRI', 'P_PMed', 'T_PRI',\
        'S_PNEU', 'Lc_PNEU','A_PNEU', 'M_PNEU', 'P_PNEU', 'P_PnMed', 'T_PNEU',\
        'C_LHS', 'C_RHS', 'C_TOP', 'C_DEG', time.time()])



class ardLogger():

    def __init__(self):
        self.ardData = []
        self.numRows = 0

    def ardLog(self,lhsS, lhsLc, lhsA, lhsMaster, lhsP, lhsMed, lhsT,\
        rhsS, rhsLc, rhsA, rhsMaster, rhsP, rhsMed, rhsT,\
        topS, topLc, topA, topMaster, topP, topMed, topT,\
        extS, extLc, extA, extMaster, extP, extMed, extT,\
        pneuS, pneuLc, pneuA, pneuMaster, pneuP, pneuMed, pneuT,\
        lhsC, rhsC, topC, degC):
        """
        Save stepCount, master cable lengths, pressure values and time 
        from pumps in a list to later save in csv.
        """
        # Take a list as an argument and unpack that?
        args = locals() # Dictionary of input arguments
        args.pop('self') # Take argument 'self' out of dictionary
        self.ardData.append([i for i in args.values()])
        # print(self.ardData[self.numRows])
        # self.numRows = self.numRows + 1

        # ardData.append([lhsS] + [lhsLc] + [lhsA] + [lhsMaster] + [lhsP] + [lhsT]\
        #     + [rhsS] + [rhsLc] + [rhsA] + [rhsMaster] + [rhsP] + [rhsT]\
        #     + [topS] + [topLc] + [topA] + [topMaster] + [topP] + [topT])
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