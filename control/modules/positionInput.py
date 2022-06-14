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
    logger1.writerow(['X', 'Y', 'Z', 'theta', 'roll', 'Timestamp', time.time()])

class posLogger():

    def __init__(self):
        self.poseData = []
        print(self.poseData)

    def posLog(self, desX, desY, desZ, theta, roll):
        print(theta)
        self.poseData.append([desX] + [desY] + [desZ] + [theta] + [roll] + [time.time()])
        
    def posSave(self):
        with open(fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])