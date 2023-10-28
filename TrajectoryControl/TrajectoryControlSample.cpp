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
	// ��ʼ��SimOneAPI
	bool inAEBState = false;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "TrajectoryControl");
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);

	int timeout = 20;
	// ���ظ����ͼ��Ϣ
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();

		// ���ģ�ⳡ��ֹͣ������������¼���˳�
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		// ��ȡ����GPS���ϰ�����Ϣ
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		// ���ģ�ⳡ����������
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			// ��ȡ��������λ�ú��ٶ�
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";

			bool insideLane = false;

			// �����ϰ���ҵ�����������������ϰ���
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
				// ��ȡ���������ϰ����ڳ����ϵ�λ��
				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

				// �ж��ϰ����Ƿ���������ǰ��
				isObstacleBehind = !(sMainVehicle >= sObstacle);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// ����������
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			double defaultDistance = 10.0;		// �ϰ���ת����ֵ
			double safeDistance = 20.0;  // ɲ����ȫ����
			double searchDistance = 5;  // �����������룬��λΪ��
			// ����ϰ�����������ǰ������ִ�б����߼�
			if (isObstacleBehind || ((safeDistance >= minDistance && insideLane))) {
					SSD::SimPoint3DVector targetPath;	// �洢Ŀ��·��
					SSD::SimStringVector nearLanes;		// ������Χ����
					SSD::SimStringVector filteredNearLanes;		// ������˺󳵵�
					SimOneAPI::GetNearLanes(mainVehiclePos, searchDistance, nearLanes);		// ������������
					// ���˵���ǰ����
					for (const SSD::SimString& laneId : nearLanes) {
						if (laneId != mainVehicleLaneId) {
							filteredNearLanes.push_back(laneId);
						}
					}
					// ����ҵ��˸����ĳ���
					if (filteredNearLanes.size() > 0) {
						printf("���ڸ�������\n");
						if (minDistance < defaultDistance) {
							printf("�ϰ������뵽��ֵ\n");
							printf("ִ�б����߼�\n");
							SSD::SimString nearestLaneId = filteredNearLanes[0];  // ��ȡ��һ������
							HDMapStandalone::MLaneInfo laneInfo;
							if (SimOneAPI::GetLaneSample(nearestLaneId, laneInfo)) {
								// ת������λ��Ϊ�ַ���
								std::string mainVehicleStr = "X: " + std::to_string(mainVehiclePos.x) +
										" Y: " + std::to_string(mainVehiclePos.y) +
										" Z: " + std::to_string(mainVehiclePos.z);

								std::cout << "��������ID: " << mainVehicleLaneId.GetString() << std::endl;
								std::cout << "Ŀ�공��ID: " << laneInfo.laneName.GetString() << std::endl;
								std::cout << "��ǰ����λ�ã�" << mainVehicleStr << std::endl;
								targetPath = laneInfo.centerLine;		// ��Ŀ�공����������ΪĿ���
								
							}
							pControl->brake = 0.3;     //ɲ��ֹͣ
							// ���㷽����ת�ǲ�Ӧ��
							pControl->steering = (double) UtilDriver::calculateSteering(targetPath, pGps.get()) * 1.5;
						}
						
						//// ���ݷ�����ת���ж�ת���ź�
						//SimOne_Data_Signal_Lights signalLights;
						//if (steering < 0) {
						//	// ��ת
						//	signalLights.signalLights = ESimOne_Signal_Light_LeftBlinker;
						//}
						//else if (steering > 0) {
						//	// ��ת
						//	signalLights.signalLights = ESimOne_Signal_Light_RightBlinker;
						//}
						//else {
						//	// ֹͣת��
						//	signalLights.signalLights = ESimOne_Signal_Light_None;
						//}

						//// ���ó����źŵ�״̬
						//if (SimOneAPI::SetSignalLights(MainVehicleId, &signalLights)) {
						//	std::cout << "�����źŵ�״̬�ɹ�" << std::endl;
						//}
						//else {
						//	std::cout << "�����źŵ�״̬ʧ��" << std::endl;
						//}
					}
					else {
						std::cout << "�����޳�����Ϣ������ɲ���������\n" << std::endl;
						HDMapStandalone::MSideState sideState;// ����һ�� MSideState �������ڽ���λ��״̬
						// ���� IsInsideLane �������м��
						bool insideLane = SimOneAPI::IsInsideLane(potentialObstaclePos, mainVehicleLaneId, sideState);// �ж��ϰ����Ƿ��ڳ�����
						if (insideLane) {
							printf("�������ж���\n");
						}
						std::cout << "�������ϰ������: " << minDistance << std::endl;
						std::cout << "�ϰ����ٶ�: " << obstacleSpeed << std::endl;
						if (safeDistance > minDistance) {
							if (obstacleSpeed < 0.01 && insideLane) {			// �ϰ���ֹͣ�ڵ�·�ϲ���
								inAEBState = false;
								printf("�ϰ���ֹͣ�ڵ�·�У�\n");
								pControl->throttle = 0.0;  // ֹͣ����
								pControl->brake = 1.0;     //ɲ��
							}
							else if (obstacleSpeed < mainVehicleSpeed && obstacleSpeed >0.01 && insideLane) {

								inAEBState = true;  // ����AEB״̬
							}
							else {
								inAEBState = false;
								printf("ɲ����ȡ������\n");
							}
						}
						else{
							inAEBState = false;
							printf("ɲ����ȡ������\n");
						}

						if (inAEBState) {
							if (mainVehicleSpeed == int(obstacleSpeed) && safeDistance <= minDistance) {
								inAEBState = false;
							}
							else{
								printf("AEBɲ����\n");
								pControl->brake = 0.8;  // ִ���ƶ�
							}
							
						}
						else {
							if (obstacleSpeed > 0.01){
								printf("���٣�����\n");
								pControl->throttle = 0.5;
								pControl->brake = 0.0;
							}
						}
					}
			}
			else {
				printf("���ϰ�����Ϣ\n");
			}
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());		// ��������������
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
