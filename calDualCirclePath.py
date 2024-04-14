import numpy as np
from math import sin, cos, atan2



def transformGlobalToLocal(originPose, globalPose):
    
    localPose = np.array()
    
    return localPose



def transformLocalToGlobal(originPose, localPose):
    
    globalPose = np.array()
    
    return globalPose




def checkFeasibility():
    
    feasibilityFlg = 0
    
    return feasibilityFlg



def calKappa():
    
    kappa1 = 0
    kappa2 = 0
    
    return kappa1, kappa2


def calLength():
    
    length1 = 0
    length2 = 0
    
    return length1, length2


def calError():
    
    error = 0
    
    return error


def calWaypoint():
    
    return






def calDualCirclePath(initPose, endPose, minTurningRadius):
    
    localPose = transformGlobalToLocal(initPose, endPose)
        
    if ~checkFeasibility(localPose, minTurningRadius):
        return
    
    calKappa()
    
    calLength()
    
    calError()
    
    calWaypoint()
    
    return




def main():
    print("calDualCirclePath")
    
    initPose = np.array(0, 0, np.deg2rad(0))
    endPose = np.array(5, 1, np.deg2rad(0))
    minTurningRadius = 5
    intervalDist = 0.2
    
    calDualCirclePath(initPose, endPose, minTurningRadius, intervalDist)
    
    

if __name__ == '__main__':
    main()