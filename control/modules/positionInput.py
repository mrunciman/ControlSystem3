import time
import csv
import os
import platform


class posLogger():

    def __init__(self):
        self.poseData = []

        if platform.system() == "Windows":
            self.parent = "C:/Users/msrun/Documents/Inflatable Robot Control/ControlSystem3/control/"
        else:
            self.parent = "/home/lannsair/Documents/DataLogs/DT Prime/"

        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "logs/positions/desired " + self.logTime + ".csv"
        self.fileName = os.path.join(self.parent, self.relative)
        # self.fileName = 'desiredLog.csv' # For test purposes
        with open(self.fileName, mode ='w', newline='') as posLog1: 
            logger1 = csv.writer(posLog1)
            logger1.writerow(['Robot X', 'Robot Y', 'Robot Z', 'Robot Roll', 'Robot Pitch', 'Robot Yaw', 'Timestamp', time.time(), "ZYX Euler angles: R = Rz(yaw) * Ry(pitch) * Rx(roll)"])

    def posLog(self, desX, desY, desZ, roll, pitch, yaw):
        self.poseData.append([desX] + [desY] + [desZ] + [roll] + [pitch] + [yaw] + [time.time()])
        
    def posSave(self):
        with open(self.fileName, 'a', newline='') as posLog2:
            positionLog2 = csv.writer(posLog2)
            for i in range(len(self.poseData)):
                positionLog2.writerow(self.poseData[i])