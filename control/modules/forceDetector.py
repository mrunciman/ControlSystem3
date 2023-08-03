import numpy as np

WINDOW_SIZE = 10




class forceDetector:

    def __init__(self):

        self.pressArray = np.zeros(WINDOW_SIZE)
        self.pressMed = np.median(self.pressArray)
        self.pressMedians = np.zeros(3)

        self.pressMedPrev = self.pressMed
        self.presMedPrevPrev = self.pressMed

        self.derivThresh = 2000
        self.deriv2Thresh = 3000

        self.conDetected = False


    def newPressMed(self, newPress):

        self.pressArray = np.roll(self.pressArray, 1)
        self.pressArray[0] = newPress
        
        self.pressMedians = np.roll(self.pressMedians, 1)
        self.pressMedians[0] = np.median(self.pressArray)

        self.presMedPrevPrev = self.pressMedPrev
        self.pressMedPrev = self.pressMed
        self.pressMed = np.median(self.pressArray)

        return self.pressMed


    def derivPress(self, timeStep = None):
        # realTimeStep = 0.01 #(newTime - prevTime)/1000
        # # Prevent div by zero errors / limit noise?
        # if realTimeStep <= 0.001:
        #     realTimeStep = 0.001
        if timeStep is None:
            timeStep = 0.01
        # pressDiff = self.pressMed - self.pressMedPrev

        # Centred finite difference is for previous timestep
        centredDeriv = (self.pressMedians[0] - self.pressMedians[2])/2*timeStep
        secondDeriv = (self.pressMedians[0] - 2*self.pressMedians[1] + self.pressMedians[2])/(timeStep**2)

        # deriv = pressDiff/realTimeStep
        # deriv2 = deriv/realTimeStep
        # Check derivatives against heuristically determined threshold values to determine contact:
        # self.conDetected = (abs(deriv) > self.derivThresh) & (abs(deriv2) > self.deriv2Thresh)
        self.conDetected = (abs(centredDeriv) > self.derivThresh) & (abs(secondDeriv) > self.deriv2Thresh)
        # 1 means actuator in tension, -1 means compression, 0 means no contact
        self.conDetected = self.conDetected*np.sign(centredDeriv)
        return self.conDetected, centredDeriv, secondDeriv


    # def initPress(self, pressIndex, pressVal):
    #     self.pressArray[pressIndex] = float(pressVal)


    # USE COLLISION ANGLE FUNCTION FROM KINEMATICS
    # def collisionAngle(self, dL, dR, dT, conL, conR, conT):
    #     # In the case of no contact:
    #     if not(conL) and not(conR) and not(conT):
    #         collAngle = None
    #     else:
    #         derivRef = max(np.absolute(np.array([dL, dR, dT])))
    #         derRatL = dL/derivRef
    #         derRatR = dR/derivRef
    #         derRatT = dT/derivRef
    #         relTensions = np.array([derRatL, derRatR, derRatT])
    #         tensionVects = self.uCables*relTensions
    #         resultant = np.array([np.sum(tensionVects[0,:]), np.sum(tensionVects[1,:]), np.sum(tensionVects[2,:])])
    #         resDir = resultant/la.norm(resultant)
    #         resDirX = resDir[0]
    #         resDirY = resDir[1]
    #         collAngle = (180/np.pi)*np.arctan2(resDirY, resDirX)

    #     return collAngle


if __name__ == "__main__":
    forceD = forceDetector()
    for x in range(4*WINDOW_SIZE):
        if (x < WINDOW_SIZE + 10):
            forceD.newPressMed(x)
        else:
            forceD.newPressMed(x*(-1)**x)

        [contactFlag, cDeriv, c2Deriv] = forceD.derivPress()

        print("Window of pressures: ", forceD.pressArray)
        print("Last three medians:  ", forceD.pressMedians)
        print("Derivatives:         ", cDeriv, c2Deriv)
