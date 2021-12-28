 #############################################################
# Calibrate arduinos for zero volume - maintain negative pressure for 4 seconds
calibL = False
calibR = False
calibT = False
calibP = False
calibA = False
# Has the mechanism been calibrated/want to run without calibration?:
calibrated = False
# Perform calibration:
while (not calibrated):
    [realStepL, pressL, timeL] = ardIntLHS.listenZero(calibL, pressL, timeL)
    # print(realStepL, pressL)
    [realStepR, pressR, timeR] = ardIntRHS.listenZero(calibR, pressR, timeR)
    # print(realStepR, pressR)
    [realStepT, pressT, timeT] = ardIntTOP.listenZero(calibT, pressT, timeT)
    # print(realStepT, pressT)
    [realStepP, pressP, timeP] = ardIntPRI.listenZero(calibP, pressP, timeP)
    # print(realStepP, calibP)
    [realStepA, pressA, timeA] = ardIntPNEU.listenReply()
    print(pressA, calibA)

    if (realStepL == "0000LHS"):
        calibL = True
    if (realStepR == "0000RHS"):
        calibR = True
    if (realStepT == "0000TOP"):
        calibT = True
    if (realStepP == "0200PRI"):
        calibP = True
    if (pressA >= pneuPress):
        calibA = True
    if (calibL * calibR * calibT * calibP * calibA == 1):
        calibrated = True
        # Send 0s instead of StepNo and pressMed as signal that calibration done
        StepNoL, StepNoR, StepNoT, StepNoP, StepNoA = 0
        pressLMed, pressRMed, pressTMed, pressPMed, pressAMed = 0

    ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, pressLMed, timeL,\
        realStepR, LcRealR, angleR, StepNoR, pressR, pressRMed,  timeR,\
        realStepT, LcRealT, angleT, StepNoT, pressT, pressTMed, timeT,\
        realStepP, LcRealP, angleP, StepNoP, pressP, pressPMed, timeP,\
        realStepA, LcRealA, angleA, StepNoA, pressA, pressAMed, timeA,\
        conLHS, conRHS, conTOP, collisionAngle)