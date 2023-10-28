#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>
#include "SimOneEvaluationAPI.h"

int main() {
	// 初始化SimOneAPI
	bool inAEBState = false;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "TrajectoryControl");
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);

	int timeout = 20;
	// 加载高清地图信息
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();

		// 如果模拟场景停止，保存评估记录并退出
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		// 获取车辆GPS和障碍物信息
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		// 如果模拟场景正在运行
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			// 获取主车辆的位置和速度
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";

			bool insideLane = false;

			// 遍历障碍物，找到距离主车辆最近的障碍物
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos); 

				if (mainVehicleLaneId == obstacleLaneId) {
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}
			}


			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
			double sObstacle = 0;
			double tObstacle = 0;

			double sMainVehicle = 0;
			double tMainVehicle = 0;

			bool isObstacleBehind = false;
			if (!potentialObstacleLaneId.Empty()) {
				// 获取主车辆和障碍物在车道上的位置
				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

				// 判断障碍物是否在主车辆前方
				isObstacleBehind = !(sMainVehicle >= sObstacle);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// 控制主车辆
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			double defaultDistance = 10.0;		// 障碍物转向阈值
			double safeDistance = 20.0;  // 刹车安全距离
			double searchDistance = 5;  // 设置搜索距离，单位为米
			// 如果障碍物在主车辆前方，先执行避障逻辑
			if (isObstacleBehind || ((safeDistance >= minDistance && insideLane))) {
					SSD::SimPoint3DVector targetPath;	// 存储目标路径
					SSD::SimStringVector nearLanes;		// 储存周围车道
					SSD::SimStringVector filteredNearLanes;		// 储存过滤后车道
					SimOneAPI::GetNearLanes(mainVehiclePos, searchDistance, nearLanes);		// 搜索附近车道
					// 过滤掉当前车道
					for (const SSD::SimString& laneId : nearLanes) {
						if (laneId != mainVehicleLaneId) {
							filteredNearLanes.push_back(laneId);
						}
					}
					// 如果找到了附近的车道
					if (filteredNearLanes.size() > 0) {
						printf("存在附近车道\n");
						if (minDistance < defaultDistance) {
							printf("障碍物达距离到阈值\n");
							printf("执行避障逻辑\n");
							SSD::SimString nearestLaneId = filteredNearLanes[0];  // 获取第一个车道
							HDMapStandalone::MLaneInfo laneInfo;
							if (SimOneAPI::GetLaneSample(nearestLaneId, laneInfo)) {
								// 转换主车位置为字符串
								std::string mainVehicleStr = "X: " + std::to_string(mainVehiclePos.x) +
										" Y: " + std::to_string(mainVehiclePos.y) +
										" Z: " + std::to_string(mainVehiclePos.z);

								std::cout << "主车车道ID: " << mainVehicleLaneId.GetString() << std::endl;
								std::cout << "目标车道ID: " << laneInfo.laneName.GetString() << std::endl;
								std::cout << "当前主车位置：" << mainVehicleStr << std::endl;
								targetPath = laneInfo.centerLine;		// 将目标车道中心线设为目标点
								
							}
							pControl->brake = 0.3;     //刹车停止
							// 计算方向盘转角并应用
							pControl->steering = (double) UtilDriver::calculateSteering(targetPath, pGps.get()) * 1.5;
						}
						
						//// 根据方向盘转角判断转向信号
						//SimOne_Data_Signal_Lights signalLights;
						//if (steering < 0) {
						//	// 左转
						//	signalLights.signalLights = ESimOne_Signal_Light_LeftBlinker;
						//}
						//else if (steering > 0) {
						//	// 右转
						//	signalLights.signalLights = ESimOne_Signal_Light_RightBlinker;
						//}
						//else {
						//	// 停止转向
						//	signalLights.signalLights = ESimOne_Signal_Light_None;
						//}

						//// 设置车辆信号灯状态
						//if (SimOneAPI::SetSignalLights(MainVehicleId, &signalLights)) {
						//	std::cout << "设置信号灯状态成功" << std::endl;
						//}
						//else {
						//	std::cout << "设置信号灯状态失败" << std::endl;
						//}
					}
					else {
						std::cout << "附近无车道信息，进行刹车或跟车。\n" << std::endl;
						HDMapStandalone::MSideState sideState;// 定义一个 MSideState 对象，用于接收位置状态
						// 调用 IsInsideLane 函数进行检查
						bool insideLane = SimOneAPI::IsInsideLane(potentialObstaclePos, mainVehicleLaneId, sideState);// 判断障碍物是否在车道上
						if (insideLane) {
							printf("车道上有东西\n");
						}
						std::cout << "主车与障碍物距离: " << minDistance << std::endl;
						std::cout << "障碍物速度: " << obstacleSpeed << std::endl;
						if (safeDistance > minDistance) {
							if (obstacleSpeed < 0.01 && insideLane) {			// 障碍物停止在道路上操作
								inAEBState = false;
								printf("障碍物停止在道路中！\n");
								pControl->throttle = 0.0;  // 停止油门
								pControl->brake = 1.0;     //刹车
							}
							else if (obstacleSpeed < mainVehicleSpeed && obstacleSpeed >0.01 && insideLane) {

								inAEBState = true;  // 进入AEB状态
							}
							else {
								inAEBState = false;
								printf("刹车被取消！！\n");
							}
						}
						else{
							inAEBState = false;
							printf("刹车被取消！！\n");
						}

						if (inAEBState) {
							if (mainVehicleSpeed == int(obstacleSpeed) && safeDistance <= minDistance) {
								inAEBState = false;
							}
							else{
								printf("AEB刹车！\n");
								pControl->brake = 0.8;  // 执行制动
							}
							
						}
						else {
							if (obstacleSpeed > 0.01){
								printf("加速！！！\n");
								pControl->throttle = 0.5;
								pControl->brake = 0.0;
							}
						}
					}
			}
			else {
				printf("无障碍物信息\n");
			}
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());		// 更新主车控制器
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
