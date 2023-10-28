#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import pySimOneIO
import math
import heapq
import sys
import time
from SimOneIOStruct import *

case_info = SimOne_Data_CaseInfo()
SimOne_Data_Gps_Test_Sync = SimOne_Data_Gps()
SimOne_Data_MainVehicle_Info_Test = SimOne_Data_MainVehicle_Info()
SimOne_Data_MainVehicle_Status_Test = SimOne_Data_MainVehicle_Status()
control = SimOne_Data_Control()
wayPoints = SimOne_Data_WayPoints()
SimOne_Data_Obstacle_Test = SimOne_Data_Obstacle()

CLOUD_PLATFORM = 0
M_PI = 3.14159265358979323846


def SampleGetNearMostLane(pos):
    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "SampleGetNearMostLane:")
    else:
        print("SampleGetNearMostLane:")
    info = pySimOneIO.getNearMostLane(pos)
    if info.exists == False:
        if CLOUD_PLATFORM:
            SoBridgeLogOutput(0, "Not exists!")
        else:
            print("Not exists!")
        return
    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "lane id:%s" % info.laneId.GetString())
    else:
        print("lane id:", info.laneId.GetString())
    return info.laneId


def apiAllStart(isJoinTimeLoop):
    SoAPIStartSimOneNode(0, 0)
    SoAPISimOneNodeReady()
    if SoAPIGetCaseInfo(case_info):
        if CLOUD_PLATFORM:
            SoBridgeLogOutput(0, "GetCaseInfo caseName: %s" % case_info.caseName)
            SoBridgeLogOutput(0, "GetCaseInfo caseId: %s" % case_info.caseId)
            SoBridgeLogOutput(0, "GetCaseInfo taskId: %s" % case_info.taskId)
            SoBridgeLogOutput(0, "GetCaseInfo sessionId: %s" % case_info.sessionId)
        else:
            print("GetCaseInfo caseName: %s" % case_info.caseName)
            print("GetCaseInfo caseId: %s" % case_info.caseId)
            print("GetCaseInfo taskId: %s" % case_info.taskId)
            print("GetCaseInfo sessionId: %s" % case_info.sessionId)

    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "SoAPIGetCaseRunStatus: %s" % SoAPIGetCaseRunStatus())
    else:
        print("SoAPIGetCaseRunStatus: %s" % SoAPIGetCaseRunStatus())

    if SoAPIGetMainVehicleList(SimOne_Data_MainVehicle_Info_Test):
        if CLOUD_PLATFORM:
            print("MainVehicle size: %s" % SimOne_Data_MainVehicle_Info_Test.size)
        else:
            SoBridgeLogOutput(0, "MainVehicle size: %s" % SimOne_Data_MainVehicle_Info_Test.size)

    while (1):
        SoAPISubMainVehicle_result = SoAPISubMainVehicle(0, isJoinTimeLoop)
        if CLOUD_PLATFORM:
            SoBridgeLogOutput(0, "SoAPISubMainVehicle_result:%s" % SoAPISubMainVehicle_result)
        else:
            print("SoAPISubMainVehicle_result:%s" % SoAPISubMainVehicle_result)
        if SoAPISubMainVehicle_result:
            break

    if SoAPIGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):
        if CLOUD_PLATFORM:
            SoBridgeLogOutput(0, "mainVehicleId:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleId)
            SoBridgeLogOutput(0, "mainVehicleStatus:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus)
        else:
            print("mainVehicleId")
            print(SimOne_Data_MainVehicle_Status_Test.mainVehicleId)
            print("mainVehicleStatus")
            print(SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus)

    ret = pySimOneIO.loadHDMap(100)
    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "Load xodr success:%s" % ret)
    else:
        print("Load xodr success:", ret)


def calculateSpeed(velX, velY, velZ):
    return math.sqrt(pow(velX, 2) + pow(velY, 2)+ pow(velZ, 2))


def planarDistance(pt1, pt2):
    return math.sqrt(pow(pt1.posX-pt2.posX, 2) + pow(pt1.posY-pt2.posY, 2))


def SampleGetLaneST(laneId, pos):
    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "SampleGetLaneST:")
    else:
        print("SampleGetLaneST:")
    stInfo = pySimOneIO.getLaneST(laneId, pos)
    if stInfo.exists == False:
        if CLOUD_PLATFORM:
            SoBridgeLogOutput(0, "Not exists!")
        else:
            print("Not exists!")
        return
    if CLOUD_PLATFORM:
        SoBridgeLogOutput(0, "[%s,%s] relative to this lane:"%(stInfo.s,stInfo.t))
    else:
        print("[s,t] relative to this lane:", stInfo.s, ",", stInfo.t)
    return stInfo.s, stInfo.t


if __name__ == '__main__':

    inAEBState = False
    isSimOneInitialized = False
    apiAllStart(True)
    SoSetDriverName(0, "AEB")

    while(1):
        if SoAPIGetCaseRunStatus() == 1:
            if CLOUD_PLATFORM:
                SoBridgeLogOutput(0, "case stop")
            else:
                print("case stop")
            break

        frame = SoAPIWait()
        if not SoAPIGetSimOneGps(SimOne_Data_Gps_Test_Sync):
            if CLOUD_PLATFORM:
                SoBridgeLogOutput(0, "Fetch Gps Failed")
            else:
                print("Fetch Gps Failed")

        if not SoAPIGetSimOneGroundTruth(SimOne_Data_Obstacle_Test):
            if CLOUD_PLATFORM:
                SoBridgeLogOutput(0, "Fetch obstacle failed")
            else:
                print("Fetch obstacle failed")

        mainVehiclePos = pySimOneIO.pySimPoint3D(SimOne_Data_Gps_Test_Sync.posX, SimOne_Data_Gps_Test_Sync.posY, SimOne_Data_Gps_Test_Sync.posZ)
        mainVehicleSpeed = calculateSpeed(SimOne_Data_Gps_Test_Sync.velX, SimOne_Data_Gps_Test_Sync.velY, SimOne_Data_Gps_Test_Sync.velZ)
        minDistance = 10000000
        potentialObstacleIndex = SimOne_Data_Obstacle_Test.obstacleSize
        mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos)
        potentialObstacleLaneId = ""

        for i in range(0, SimOne_Data_Obstacle_Test.obstacleSize):
            obstaclePos = pySimOneIO.pySimPoint3D(SimOne_Data_Obstacle_Test.obstacle[i].posX, SimOne_Data_Obstacle_Test.obstacle[i].posY, SimOne_Data_Obstacle_Test.obstacle[i].posZ)
            obstacleLaneId = SampleGetNearMostLane(obstaclePos)
            if mainVehicleLaneId == obstacleLaneId:
                obstacleDistance = planarDistance(mainVehiclePos, obstaclePos)

                if obstacleDistance < minDistance:
                    minDistance = obstacleDistance
                    potentialObstacleIndex = i
                    potentialObstacleLaneId = obstacleLaneId

        potentialObstacle = SimOne_Data_Obstacle_Test.obstacle[potentialObstacleIndex]
        obstacleSpeed = calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ)

        potentialObstaclePos = pySimOneIO.pySimPoint3D(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ)

        sObstalce = 0
        tObstacle = 0
        sMainVehicle = 0
        tMainVehicle = 0
        isObstalceBehind = False

        if potentialObstacleLaneId:
            sObstalce, tObstacle = SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos)
            sMainVehicle, tMainVehicle = SampleGetLaneST(potentialObstacleLaneId, mainVehiclePos)
            isObstalceBehind = False if sMainVehicle >= sObstalce else True

        SoGetDriverControl(0, control)

        if isObstalceBehind:
            defaultDistance = 10
            timeToCollision = abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed)
            defautlTimeToCollision = 3.4
            if -timeToCollision < defautlTimeToCollision and timeToCollision < 0:
                inAEBState = True
                control.brake = mainVehicleSpeed * 3.6 * 0.65 + 0.20

            if inAEBState:
                control.throttle = 0
        SoApiSetDrive(0, control)

        SoAPINextFrame(frame)



