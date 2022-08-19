import time
import csv
import os

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/positions/desired " + logTime + ".csv"
fileName = os.path.join(parent, relative)
with open(fileName, mode ='w', newline='') as posLog1: 
    logger1 = csv.writer(posLog1)
    logger1.writerow(['X', 'Vel', 'U1', 'U2', 'Fhat', 'Timestamp', time.time()])

class posLogger():

    def __init__(self):
        self.poseData = []

    def posLog(self, desX, desY, desZ, inclination, azimuth):
        self.poseData.append([desX] + [desY] + [desZ] + [inclination] + [azimuth] + [time.time()])
        
    def posSave(self):
        with open(fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])