import time
import csv
import os

# location = os.path.dirname(__file__)
# parent = os.path.dirname(location)
# parent = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/DataLogs/DT_Prime"
# logTime = time.strftime("%Y-%m-%d %H-%M-%S")
# relative = "logs/positions/desired " + logTime + ".csv"
# fileName = os.path.join(parent, relative)
# with open(fileName, mode ='w', newline='') as posLog1: 
#     logger1 = csv.writer(posLog1)
#     logger1.writerow(['X', 'Y', 'Z', 'inclination', 'azimuth', 'Timestamp', time.time()])

class posLogger():

    def __init__(self):
        self.poseData = []

        self.parent = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/DataLogs/DT_Prime"
        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "logs/positions/desired " + self.logTime + ".csv"
        self.fileName = os.path.join(self.parent, self.relative)
        with open(self.fileName, mode ='w', newline='') as posLog1: 
            logger1 = csv.writer(posLog1)
            logger1.writerow(['X', 'Y', 'Z', 'inclination', 'azimuth', 'Timestamp', time.time()])

    def posLog(self, desX, desY, desZ, inclination, azimuth):
        self.poseData.append([desX] + [desY] + [desZ] + [inclination] + [azimuth] + [time.time()])
        
    def posSave(self):
        with open(self.fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])