import numpy as np
from navpy import wrapToPi
from math import pi, sin, cos, atan2, sqrt, pow, dist


def transformGlobalToLocal(originPose, globalPose):
    
    if np.size(globalPose) == 3:
        localPose = np.array([
             cos(originPose[2]) * (globalPose[0] - originPose[0]) + sin(originPose[2]) * (globalPose[1] - originPose[1]),
            -sin(originPose[2]) * (globalPose[0] - originPose[0]) + cos(originPose[2]) * (globalPose[1] - originPose[1]),
            wrapToPi(globalPose[2] - originPose[2])
        ])
        
    elif np.size(globalPose) == 2:
        localPose = np.array([
             cos(originPose[2]) * (globalPose[0] - originPose[0]) + sin(originPose[2]) * (globalPose[1] - originPose[1]),
            -sin(originPose[2]) * (globalPose[0] - originPose[0]) + cos(originPose[2]) * (globalPose[1] - originPose[1])
        ])
        
    else:
        localPose = 0
    
    return localPose


def transformLocalToGlobal(originPose, localPose):
    
    if np.size(localPose) == 3:
        globalPose = np.array([
            cos(originPose[2]) * localPose[0] - sin(originPose[2]) * localPose[1] + originPose[0],
            sin(originPose[2]) * localPose[0] + cos(originPose[2]) * localPose[1] + originPose[1],
            originPose[2] + localPose[2]
        ])
        
    elif np.size(localPose) == 2:
        globalPose = np.array([
            cos(originPose[2]) * localPose[0] - sin(originPose[2]) * localPose[1] + originPose[0],
            sin(originPose[2]) * localPose[0] + cos(originPose[2]) * localPose[1] + originPose[1]
        ])
        
    else:
        globalPose = 0
            
    return globalPose


def checkFeasibility(localPose, minTurningRadius):
        
    sectorCenterPnt = np.array([
        minTurningRadius * abs(sin(localPose[2])),
        minTurningRadius * np.sign(localPose[2]) * (1 - cos(localPose[2]))
    ])
    
    sideCircle1Pnt = np.array([
        sectorCenterPnt[0] - 2 * minTurningRadius * abs(sin(localPose[2])),
        sectorCenterPnt[1] + 2 * minTurningRadius * np.sign(localPose[2]) * cos(localPose[2])
    ])
    
    sideCircle2Pnt = np.array([
        sectorCenterPnt[0],
        sectorCenterPnt[1] - 2 * minTurningRadius * np.sign(localPose[2])
    ])
    
    dist1 = dist(np.array([sideCircle1Pnt[0], localPose[0]]), np.array([sideCircle1Pnt[1], localPose[1]]))
    
    dist2 = dist(np.array([sideCircle2Pnt[0], localPose[0]]), np.array([sideCircle2Pnt[1], localPose[1]]))
    
    dist3 = dist(np.array([sectorCenterPnt[0], localPose[0]]), np.array([sectorCenterPnt[1], localPose[1]]))
    
    localX = cos(localPose[2] / 2) * (localPose[0] - sectorCenterPnt[0]) + sin(localPose[2] / 2) * (localPose[1] - sectorCenterPnt[1])
    
    print(sectorCenterPnt)
    print(sideCircle1Pnt)
    print(sideCircle2Pnt)
    print(dist1)
    print(dist2)
    print(dist3)
    
    return dist1 >= 2 * minTurningRadius and dist2 >= 2 * minTurningRadius and dist3 <= 2 * minTurningRadius and localX >= 0


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






def calDualCirclePath(initPose, endPose, minTurningRadius, intervalDist):
    
    localPose = transformGlobalToLocal(initPose, endPose)
        
    print(checkFeasibility(localPose, minTurningRadius))
        
    if not checkFeasibility(localPose, minTurningRadius):
        return
    
    calKappa()
    
    calLength()
    
    calError()
    
    calWaypoint()
    
    return




def main():
    print("calDualCirclePath")
    
    initPose = np.array([0, 0, np.deg2rad(0.1)])
    endPose = np.array([5, 1, np.deg2rad(0)])
    minTurningRadius = 5
    intervalDist = 0.2
    
    calDualCirclePath(initPose, endPose, minTurningRadius, intervalDist)
    
    

if __name__ == '__main__':
    main()