
import numpy as np


class collideDetect:
    
    def __init__(self):
        self.press1
        self.press2
        self.press3
        self.press4
        self.press5
        self.press6
        self.press7
        self.press8
        self.press9
        self.press10
        self.pressArray = np.array([self.press1, self.press2, self.press3, self.press4, self.press5\
            self.press6, self.press7, self.press8, self.press9, self.press10])
        self.pressMed = np.median(self.pressArray)

    def newPressMed(self, newPress):
        self.press10 = self.press9
        self.press9 = self.press8
        self.press8 = self.press7
        self.press7 = self.press6
        self.press6 = self.press5
        self.press5 = self.press4
        self.press4 = self.press3
        self.press3 = self.press2
        self.press2 = self.press1
        self.press1 = newPress
        self.pressArray = np.array([self.press1, self.press2, self.press3, self.press4, self.press5\
            self.press6, self.press7, self.press8, self.press9, self.press10])
        self.pressMed = np.median


    