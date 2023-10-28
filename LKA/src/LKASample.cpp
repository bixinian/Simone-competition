#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "SampleGetNearMostLane.h"
#include "SampleGetLaneST.h"
#include "SimOneEvaluationAPI.h"
#include <iostream>
#include <memory>

int main()
{
	// ��ʼ��һЩ�����ͱ�־
	bool inAEBState = false; // �Ƿ����Զ������ƶ���Automatic Emergency Braking��AEB��״̬
	int timeout = 20; // ���ص�ͼ�ĳ�ʱʱ�䣨����Ϊ��λ��
	bool isSimOneInitialized = false; // �Ƿ��ѳ�ʼ��SimOneAPI
	const char* MainVehicleId = "0"; // ��������ID
	bool isJoinTimeLoop = true; // �Ƿ����ʱ��ѭ��
	
	// ��ʼ��SimOneAPI
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "LKA"); // ������������ʻԱ����Ϊ"LKA"��Lane Keeping Assist��
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	while (true) {
		// ���ظ����ͼ������ɹ����أ����˳�ѭ��
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	// �洢�����·����
	SSD::SimPoint3DVector inputPoints;
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();

	// ��ȡ��������·����
	if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get())) {
		for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
			SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
		return -1;
	}

	// �洢Ŀ��·��
	SSD::SimPoint3DVector targetPath;

	if (pWayPoints->wayPointsSize >= 2) {
		SSD::SimVector<int> indexOfValidPoints;

		// ����·��
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
			return -1;
		}
	}
	else if (pWayPoints->wayPointsSize == 1) {
		// ���ֻ��һ��·���㣬��Ŀ��·������Ϊ��ӽ��ĳ���������
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
			return -1;
		}
		else {
			targetPath = laneInfoInit.centerLine;
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
		return -1;
	}

	while (true) {
		int frame = SimOneAPI::Wait();

		// ���ģ�ⳡ��ֹͣ������������¼���˳�
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			// ��ȡ������λ�ú��ٶ�
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";

			// �����ϰ���ҵ�����������ϰ���
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
			double sObstacle = 0.;
			double tObstacle = 0.;

			double sMainVehicle = 0.;
			double tMainVehicle = 0.;

			bool isObstacleBehind = false;

			if (!potentialObstacleLaneId.Empty()) {
				// ��ȡ���������ϰ����ڳ����ϵ�λ��
				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

				// ����ϰ����Ƿ�����������
				isObstacleBehind = !(sMainVehicle >= sObstacle);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// ʹ��SimOneDriver����������
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// �����������ķ�ʽ�����²��ֱ�ע�͵�����Ϊʹ��SimOneDriver���ƣ�
			/*
			pControl->throttle = 0.12f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
			*/

			if (isObstacleBehind) {
				double defaultDistance = 10.; // Ĭ�ϵİ�ȫ����
				double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				double defaultTimeToCollision = 3.4; // Ĭ�ϵ���ײʱ����ֵ

				if (-timeToCollision < defaultTimeToCollision && timeToCollision < 0) {
					inAEBState = true;
					pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20); // �������Ӧ�ý����ƶ�
				}

				if (inAEBState) {
					pControl->throttle = 0.f; // �������AEB״̬��ȡ����������
				}
			}

			// ���㷽����ת�ǲ�Ӧ��
			double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			pControl->steering = (float)steering;

			// �����������Ŀ��Ʋ���
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		// ������һ֡��ģ��
		SimOneAPI::NextFrame(frame);
	}

	return 0;
}
