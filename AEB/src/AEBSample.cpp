#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>
#include "SimOneEvaluationAPI.h"

int main() {
	// 初始化SimOneAPI
	bool inAEBState = false;  // 标志是否处于AEB（自动紧急制动）状态
	bool isSimOneInitialized = false;  // 标志SimOne是否已初始化
	const char* MainVehicleId = "0";  // 主车辆的唯一标识符
	bool isJoinTimeLoop = true;  // 是否加入时间循环
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "AEB");  // 设置主车辆驾驶员名称为"AEB"
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);  // 设置主车辆的驾驶模式为API控制
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);  // 初始化评估服务并使用本地数据

	int timeout = 20;
	// 加载高清地图信息，等待直到加载完成
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();  // 等待模拟环境下一帧的数据

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

			double minDistance = std::numeric_limits<double>::max();  // 初始化距离最近障碍物的最大距离
			int potentialObstacleIndex = pObstacle->obstacleSize;  // 初始化最近障碍物的索引
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);  // 获取主车辆所在的车道标识符
			SSD::SimString potentialObstacleLaneId = "";  // 初始化最近障碍物所在的车道标识符
			bool inbehind = false;							// 前方是否存在障碍物
			// 遍历障碍物，找到距离主车辆最近的障碍物
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos); // 得到障碍物距离最近的车道ID

				HDMapStandalone::MSideState sideState;// 定义一个 MSideState 对象，用于接收位置状态
				// 调用 IsInsideLane 函数进行检查
				bool insideLane = SimOneAPI::IsInsideLane(obstaclePos, mainVehicleLaneId, sideState);// 判断障碍物是否在车道上

				if (insideLane) {  // 检查障碍物是否与主车辆在同一车道
					inbehind = true;
					printf("前面有东西\n");
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);
					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;  // 更新最近距离
						potentialObstacleIndex = (int)i;  // 更新最近障碍物的索引
						potentialObstacleLaneId = obstacleLaneId;  // 更新最近障碍物所在车道的标识符
					}
				}
				else
				{
					inbehind = false;
					printf("前面没有东西\n");
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			// 得到障碍物速度
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

			//SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
			//double sObstacle = 0;
			//double tObstacle = 0;

			//double sMainVehicle = 0;
			//double tMainVehicle = 0;

			//bool isObstacleBehind = false;
			//if (!potentialObstacleLaneId.Empty()) {
			//	// 获取主车辆和障碍物在车道上的位置
			//	SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
			//	SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

			//	// 判断障碍物是否在主车辆前方
			//	isObstacleBehind = !(sMainVehicle >= sObstacle);
			//}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// 控制主车辆
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// 如果障碍物在主车辆前方，执行AEB逻辑
			if (inbehind) {
				double defaultDistance = 11.0;  // 默认安全距离
				//double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				//double defaultTimeToCollision = 3;  // 默认碰撞时间
				if (defaultDistance > minDistance) {
					if (obstacleSpeed == 0) {			// 障碍物停止在道路则
						inAEBState = false;
						printf("障碍物停止！\n");
						pControl->throttle = 0.0;  // 停止油门
						pControl->brake = 1.0;     //刹车停止
					}
					else{
					inAEBState = true;  // 进入AEB状态
					}
				}
				else{
					inAEBState = false;
					printf("刹车被取消！！\n");
				}

				if (inAEBState) {
					printf("AEB刹车！\n");
					pControl->brake = (float)(mainVehicleSpeed * 3.0 * 0.65 + 0.20);  // 执行制动
				}
				else{
					pControl->throttle = 0.5;
				}
			}

			SimOneAPI::SetDrive(MainVehicleId, pControl.get());  // 更新主车辆的控制指令
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);  // 进入下一帧模拟
	}
	return 0;
}
