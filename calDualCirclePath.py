import numpy as np
from navpy import wrapToPi
from math import sin, cos, atan2


def transformPoseGlobalToLocal(originPose, globalPose):
    
    localPose = np.array([
        cos(originPose[2]) * (globalPose[0] - originPose[0]) + sin(originPose[2]) * (globalPose[1] - originPose[1]),
        -sin(originPose[2]) * (globalPose[0] - originPose[0]) + cos(originPose[2]) * (globalPose[1] - originPose[1]),
        wrapToPi(globalPose[2] - originPose[2])
    ])
    
    return localPose


def transformPoseLocalToGlobal(originPose, localPose):
    
    globalPose = np.array([
        cos(originPose[2]) * localPose[0] - sin(originPose[2]) * localPose[1] + originPose[0],
        sin(originPose[2]) * localPose[0] + cos(originPose[2]) * localPose[1] + originPose[1],
        originPose[2] + localPose[2]
    ])
            
    return globalPose


def checkFeasibility(x, y, psi, maxKappa):
    
    minTurningRadius = 1/maxKappa
    
    sectorCenterPnt = np.array([
        minTurningRadius * abs(sin(psi)),
        minTurningRadius * np.sign(psi) * (1 - cos(psi))
    ])
    
    sideCircle1Pnt = np.array([
        sectorCenterPnt[0] - 2 * minTurningRadius * abs(sin(psi)),
        sectorCenterPnt[1] + 2 * minTurningRadius * np.sign(psi) * cos(psi)
    ])
    
    sideCircle2Pnt = np.array([
        sectorCenterPnt[0],
        sectorCenterPnt[1] - 2 * minTurningRadius * np.sign(psi)
    ])
    
    dist1 = ((x - sideCircle1Pnt[0]) ** 2 + (y - sideCircle1Pnt[1]) ** 2) ** 0.5
    dist2 = ((x - sideCircle2Pnt[0]) ** 2 + (y - sideCircle2Pnt[1]) ** 2) ** 0.5
    dist3 = ((x - sectorCenterPnt[0]) ** 2 + (y - sectorCenterPnt[1]) ** 2) ** 0.5
    localX = cos(psi / 2) * (x - sectorCenterPnt[0]) + sin(psi / 2) * (y - sectorCenterPnt[1])
    
    return dist1 >= 2 * minTurningRadius and dist2 >= 2 * minTurningRadius and dist3 <= 2 * minTurningRadius and localX >= 0


def calKappaLimit(x, y, psi, maxKappa):
    
    if atan2(y, x) >= psi / 2:
        kappa1Limit = np.array([maxKappa, calKappa1(-maxKappa, maxKappa, x, y, psi)])
        kappa2Limit = np.array([calKappa2(maxKappa, maxKappa, x, y, psi), -maxKappa])
        
    else:
        kappa1Limit = np.array([calKappa1(maxKappa, maxKappa, x, y, psi), -maxKappa])
        kappa2Limit = np.array([maxKappa, calKappa2(-maxKappa, maxKappa, x, y, psi)])
    
    return kappa1Limit, kappa2Limit


def calKappa1(kappa2, maxKappa, x, y, psi):
    
    kappa1 = np.clip(2 * (kappa2 * y + cos(psi) - 1) / (kappa2 * (x ** 2 + y ** 2) - 2 * (x * sin(psi) - y * cos(psi))), -maxKappa, maxKappa)
    
    return kappa1


def calKappa2(kappa1, maxKappa, x, y, psi):
    
    kappa2 = np.clip(2 * (kappa1 * (x * sin(psi) - y * cos(psi)) + cos(psi) - 1) / (kappa1 * (x ** 2 + y ** 2) - 2 * y), -maxKappa, maxKappa)
            
    return kappa2


def calKappa(kappa1Limit, kappa2Limit, x, y, psi):
    
    kappa1 = np.clip(2 * (y - x * sin(psi / 2) + y * cos(psi / 2)) / (x ** 2 + y ** 2), kappa1Limit[1], kappa1Limit[0])
    kappa2 = np.clip(2 * (kappa1 * (x * sin(psi) - y * cos(psi)) + cos(psi) - 1) / (kappa1 * ((x ** 2 + y ** 2)) - 2 * y), kappa2Limit[1], kappa2Limit[0])
            
    return kappa1, kappa2


def calLength(kappa1, kappa2, x, y, psi):
    
    psi1 = atan2((x - sin(psi) / kappa2) * (np.sign(1 / kappa1 - 1 / kappa2)), (1 / kappa1 - y - cos(psi) / kappa2) * (np.sign(1 / kappa1 - 1 / kappa2)))
    psi2 = psi - psi1
    length1 = psi1 / kappa1
    length2 = psi2 / kappa2
    
    return length1, length2


def calWaypoint(initPose, kappa1, kappa2, length1, length2, intervalDist):
    
    waypoint = []
    
    for s1 in np.arange(0, length1, intervalDist):
        pose = calWaypointPose(kappa1, s1)
        pose = transformPoseLocalToGlobal(initPose, pose)
        waypoint.append(pose)
        
    midPose = calWaypointPose(kappa1, length1)
    midPose = transformPoseLocalToGlobal(initPose, midPose)
            
    for s2 in np.arange(s1 + intervalDist - length1, length2, intervalDist):
        pose = calWaypointPose(kappa2, s2)
        pose = transformPoseLocalToGlobal(midPose, pose)
        waypoint.append(pose)
        
    endPose = calWaypointPose(kappa2, length2)
    endPose = transformPoseLocalToGlobal(midPose, endPose)
    waypoint.append(endPose)
            
    return waypoint


def calWaypointPose(kappa, length):
    
    psi = kappa * length
    x = sin(psi) / kappa
    y = (1 - cos(psi)) / kappa
    
    return np.array([x, y, psi])


def calDualCirclePath(initPose, endPose, maxKappa):
    
    [x, y, psi] = transformPoseGlobalToLocal(initPose, endPose)

    if not checkFeasibility(x, y, psi, maxKappa):
        return 0, 0, 0, 0
    
    kappa1Limit, kappa2Limit = calKappaLimit(x, y, psi, maxKappa)
        
    kappa1, kappa2 = calKappa(kappa1Limit, kappa2Limit, x, y, psi)
        
    length1, length2 = calLength(kappa1, kappa2, x, y, psi)
            
    return kappa1, kappa2, length1, length2


def main():
    print("calDualCirclePath")
    
    initPose = np.array([0, 0, np.deg2rad(0)])
    endPose = np.array([5, 1, np.deg2rad(0.1)])
    maxKappa = 0.2
    intervalDist = 0.2
    
    kappa1, kappa2, length1, length2 = calDualCirclePath(initPose, endPose, maxKappa)
    
    waypoint = calWaypoint(initPose, kappa1, kappa2, length1, length2, intervalDist)
    
    print(kappa1)
    print(kappa2)
    print(length1)
    print(length2)
    print(waypoint)


if __name__ == '__main__':
    main()