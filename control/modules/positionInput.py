import time
import csv
import os

location = os.path.dirname(__file__)
parent = os.path.dirname(location)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/positions/desired " + logTime + ".csv"
parent = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/MRC/Energy Shaping/DATA_COLLECTION/logs/energy shaping outputs"
relative = "energy " + logTime + ".csv"
fileName = os.path.join(parent, relative)
with open(fileName, mode ='w', newline='') as posLog1: 
    logger1 = csv.writer(posLog1)
    logger1.writerow(['X', 'Vel', 'U1', 'U2', 'Fhat', 'Timestamp', 'x_d', time.time()])

class posLogger():

    def __init__(self, desired_x):
        self.poseData = []
        self.x_d = desired_x

    def posLog(self, desX, desY, desZ, inclination, azimuth):
        self.poseData.append([desX] + [desY] + [desZ] + [inclination] + [azimuth] + [time.time()] + [self.x_d])
        
    def posSave(self):
        with open(fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])