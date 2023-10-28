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
	// ��ʼ��SimOneAPI
	bool inAEBState = false;  // ��־�Ƿ���AEB���Զ������ƶ���״̬
	bool isSimOneInitialized = false;  // ��־SimOne�Ƿ��ѳ�ʼ��
	const char* MainVehicleId = "0";  // ��������Ψһ��ʶ��
	bool isJoinTimeLoop = true;  // �Ƿ����ʱ��ѭ��
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "AEB");  // ������������ʻԱ����Ϊ"AEB"
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);  // �����������ļ�ʻģʽΪAPI����
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);  // ��ʼ����������ʹ�ñ�������

	int timeout = 20;
	// ���ظ����ͼ��Ϣ���ȴ�ֱ���������
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();  // �ȴ�ģ�⻷����һ֡������

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

			double minDistance = std::numeric_limits<double>::max();  // ��ʼ����������ϰ����������
			int potentialObstacleIndex = pObstacle->obstacleSize;  // ��ʼ������ϰ��������
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);  // ��ȡ���������ڵĳ�����ʶ��
			SSD::SimString potentialObstacleLaneId = "";  // ��ʼ������ϰ������ڵĳ�����ʶ��
			bool inbehind = false;							// ǰ���Ƿ�����ϰ���
			// �����ϰ���ҵ�����������������ϰ���
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos); // �õ��ϰ����������ĳ���ID

				HDMapStandalone::MSideState sideState;// ����һ�� MSideState �������ڽ���λ��״̬
				// ���� IsInsideLane �������м��
				bool insideLane = SimOneAPI::IsInsideLane(obstaclePos, mainVehicleLaneId, sideState);// �ж��ϰ����Ƿ��ڳ�����

				if (insideLane) {  // ����ϰ����Ƿ�����������ͬһ����
					inbehind = true;
					printf("ǰ���ж���\n");
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);
					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;  // �����������
						potentialObstacleIndex = (int)i;  // ��������ϰ��������
						potentialObstacleLaneId = obstacleLaneId;  // ��������ϰ������ڳ����ı�ʶ��
					}
				}
				else
				{
					inbehind = false;
					printf("ǰ��û�ж���\n");
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			// �õ��ϰ����ٶ�
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

			//SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
			//double sObstacle = 0;
			//double tObstacle = 0;

			//double sMainVehicle = 0;
			//double tMainVehicle = 0;

			//bool isObstacleBehind = false;
			//if (!potentialObstacleLaneId.Empty()) {
			//	// ��ȡ���������ϰ����ڳ����ϵ�λ��
			//	SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
			//	SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

			//	// �ж��ϰ����Ƿ���������ǰ��
			//	isObstacleBehind = !(sMainVehicle >= sObstacle);
			//}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// ����������
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// ����ϰ�����������ǰ����ִ��AEB�߼�
			if (inbehind) {
				double defaultDistance = 11.0;  // Ĭ�ϰ�ȫ����
				//double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				//double defaultTimeToCollision = 3;  // Ĭ����ײʱ��
				if (defaultDistance > minDistance) {
					if (obstacleSpeed == 0) {			// �ϰ���ֹͣ�ڵ�·��
						inAEBState = false;
						printf("�ϰ���ֹͣ��\n");
						pControl->throttle = 0.0;  // ֹͣ����
						pControl->brake = 1.0;     //ɲ��ֹͣ
					}
					else{
					inAEBState = true;  // ����AEB״̬
					}
				}
				else{
					inAEBState = false;
					printf("ɲ����ȡ������\n");
				}

				if (inAEBState) {
					printf("AEBɲ����\n");
					pControl->brake = (float)(mainVehicleSpeed * 3.0 * 0.65 + 0.20);  // ִ���ƶ�
				}
				else{
					pControl->throttle = 0.5;
				}
			}

			SimOneAPI::SetDrive(MainVehicleId, pControl.get());  // �����������Ŀ���ָ��
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);  // ������һ֡ģ��
	}
	return 0;
}
