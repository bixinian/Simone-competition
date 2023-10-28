#include <iostream>
#include <memory>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>

#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "UtilMath.hpp"
#include "UtilUnit.hpp"
#include "UtilDriver.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"
#include "SimOneEvaluationAPI.h"
// 枚举表示AVP系统的状态
enum AVPStatus
{
	eApproaching,
	eForwarding,
	eBrakingPrep,
	eReversing,
	eFinishing,
	eWaiting,
	eLeaving,
	eEnded,
	eUnknown
};

/*!
 * @function FindTargetParkingSpace
 * @brief 查找目标停车位
 * @param parkingSpaces: 所有停车位的列表
 * @param obstacles: 障碍物信息的指针
 * @param[out] targetParkingSpace: 找到的目标停车位
 */
void FindTargetParkingSpace(const SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaces,
	const SimOne_Data_Obstacle* obstacles, HDMapStandalone::MParkingSpace& targetParkingSpace)
{
	// 存储可用停车位的索引
	std::vector<size_t> availableParkingSpaceIndices;

	// 遍历所有停车位
	for (size_t i = 0; i < parkingSpaces.size(); ++i)
	{
		auto& parkingSpace = parkingSpaces[i];
		auto& vertices = parkingSpace.boundaryKnots;
		bool occupied = false;

		// 检查停车位是否被障碍物占用
		for (size_t j = 0; j < obstacles->obstacleSize; ++j)
		{
			auto& obstacle = obstacles->obstacle[j];
			SSD::SimPoint3D pos(obstacle.posX, obstacle.posY, obstacle.posZ);

			// 如果障碍物在停车位内部
			if (UtilMath::InRectangleMargin(pos, vertices[0], vertices[2]))
			{
				std::cout << "ParkingSpace[" << i << "] occupied!" << std::endl;
				occupied = true;
				break;
			}
		}

		// 如果停车位没有被占用，将其索引添加到可用列表中+
		if (!occupied)
		{
			availableParkingSpaceIndices.push_back(i);
		}
	}

	// 选择第一个可用的停车位作为目标停车位
	targetParkingSpace = parkingSpaces[availableParkingSpaceIndices[0]];
}

/*!
 * @function RearrangeKnots
 * @brief 重新排列停车位的边界节点，使它们从左上角的节点开始
 * @param space: 目标停车位
 */
void RearrangeKnots(HDMapStandalone::MParkingSpace& space)
{
	size_t knotSize = space.boundaryKnots.size();
	SSD::SimPoint2D heading{ space.heading.x, space.heading.y };
	SSD::SimPoint3DVector arrangedBoundaryKnots(knotSize);
	double headingErrorThresholdDegree = 10.;

	// 寻找合适的排列方式
	for (size_t i = 0; i < knotSize; ++i)
	{
		SSD::SimPoint2D dir = { space.boundaryKnots[i + 1].x - space.boundaryKnots[i].x,
								space.boundaryKnots[i + 1].y - space.boundaryKnots[i].y };
		double angle = UtilMath::Angle(heading, dir);

		if (angle > 0. && angle < UtilUnit::DegreeToRad(headingErrorThresholdDegree))
		{
			for (size_t j = 0; j < knotSize; ++j)
			{
				size_t index = i + j + 2 < knotSize ? i + j + 2 : i + j + 2 - knotSize;
				arrangedBoundaryKnots[j] = space.boundaryKnots[index];
			}
			break;
		}
	}

	// 更新停车位的边界节点
	for (size_t i = 0; i < knotSize; ++i)
	{
		space.boundaryKnots[i] = arrangedBoundaryKnots[i];
	}
}

/*!
 * @function InParkingSpace
 * @brief 判断车辆是否位于停车位内
 * @param pos: 车辆位置（后部中心）
 * @param theta: 车辆的朝向（弧度）
 * @param length: 车辆长度
 * @param width: 车辆宽度
 * @param space: 停车位信息
 * @return True表示车辆的任何顶点位于停车位内，否则为False
 */
bool InParkingSpace(const SSD::SimPoint3D& pos, const double theta, const double length, const double width, const HDMapStandalone::MParkingSpace& space)
{
	// 计算车辆的四个顶点位置
	SSD::SimPoint3D p1(pos.x + width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p2(pos.x + width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p3(pos.x - width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p4(pos.x - width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);

	// 判断任何一个顶点是否在停车位内
	return UtilMath::InRectangleMargin(p1, space.boundaryKnots[0], space.boundaryKnots[2]) ||
		UtilMath::InRectangleMargin(p2, space.boundaryKnots[0], space.boundaryKnots[2]) ||
		UtilMath::InRectangleMargin(p3, space.boundaryKnots[0], space.boundaryKnots[2]) ||
		UtilMath::InRectangleMargin(p4, space.boundaryKnots[0], space.boundaryKnots[2]);
}

int main(int argc, char* argv[])
{
	// 创建指向GPS、障碍物和路径点数据的智能指针
	std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
	std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
	std::unique_ptr<SimOne_Data_WayPoints> wayPointsPtr = std::make_unique<SimOne_Data_WayPoints>();

	bool leaveAfterParked = true;
	bool isJoinTimeLoop = true;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);

	SimOneAPI::SetDriverName(0, "AVP");
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	int timeoutSecond = 20;
	if (!SimOneAPI::LoadHDMap(timeoutSecond))
	{
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}

	SSD::SimPoint3D startPoint;
	if (SimOneAPI::GetGps(0, gpsPtr.get()))
	{
		startPoint.x = gpsPtr->posX;
		startPoint.y = gpsPtr->posY;
		startPoint.z = gpsPtr->posZ;
	}
	else
	{
		std::cout << "Fetch GPS failed" << std::endl;
	}

	/* 2. 获取停车位信息 */
	SSD::SimVector<HDMapStandalone::MParkingSpace> parkingSpaces;
	SimOneAPI::GetParkingSpaceList(parkingSpaces);

	// 输出停车位信息
	for (size_t i = 0; i < parkingSpaces.size(); ++i)
	{
		auto& parkingSpace = parkingSpaces[i];
		auto& vertices = parkingSpace.boundaryKnots;
		std::cout << "ParkingSpace[" << i << "]: " << std::endl;
		for (size_t j = 0; j < 4; ++j)
		{
			std::cout << "[" << vertices[j].x << ", " << vertices[j].y << "]" << std::endl;
		}
	}
	std::cout << std::endl;

	/* 3. 获取障碍物信息 */
	SimOneAPI::GetGroundTruth(MainVehicleId, obstaclesPtr.get());
	for (size_t i = 0; i < obstaclesPtr->obstacleSize; ++i)
	{
		std::cout << "Obstacles[" << i << "]: " << std::endl;
		auto& obstacle = obstaclesPtr->obstacle[i];
		std::cout << "[" << obstacle.posX << ", " << obstacle.posY << "]" << std::endl;
	}
	std::cout << std::endl;

	/* 4. 确定目标停车位 */
	HDMapStandalone::MParkingSpace targetParkingSpace;
	FindTargetParkingSpace(parkingSpaces, obstaclesPtr.get(), targetParkingSpace);

	// 重新排列目标停车位的边界节点
	RearrangeKnots(targetParkingSpace);
	std::cout << "Rearranged parking space:" << std::endl;

	for (size_t i = 0; i < targetParkingSpace.boundaryKnots.size(); ++i)
	{
		std::cout << "[" << targetParkingSpace.boundaryKnots[i].x << ", " << targetParkingSpace.boundaryKnots[i].y << "]" << std::endl;
#if AVP_LOG
		AVPLog::getInstance().addData(targetParkingSpace.boundaryKnots[i].x);
		AVPLog::getInstance().addData(targetParkingSpace.boundaryKnots[i].y);
#endif // AVP_LOG
	}
	std::cout << std::endl;
#if AVP_LOG
	AVPLog::getInstance().addNewLine();
#endif // AVP_LOG

	/* 5. 获取终点 */
	while (!SimOneAPI::GetWayPoints(MainVehicleId, wayPointsPtr.get()))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	auto lastWayPoint = wayPointsPtr->wayPoints[wayPointsPtr->wayPointsSize - 1];
	SSD::SimPoint3D terminalPoint{ lastWayPoint.posX, lastWayPoint.posY, 0. };

	/* 6. 规划路径 */
	VehicleParam veh;
	veh.Lr = 0.88;
	veh.Lf = 1.;
	veh.L = 2.9187;

	AVPPlanner planner(targetParkingSpace, startPoint, terminalPoint, veh, leaveAfterParked);
	planner.plan();

	/* 7. 控制 */
	AVPStatus status = AVPStatus::eForwarding;

	bool brakingPrepStarted = false;
	bool reversingStarted = false;
	bool finishingStarted = false;
	bool waitingStarted = false;
	double brakingPrepTime = 1.;
	double waitingTimeSecond = 3.;
	auto startTime = std::chrono::system_clock::now();
	double headingErrorThresholdDegree = 1.0;
	double parkingEndDistanceThreshold = 0.5;
	double leavingEndDistanceThreshold = 0.5;

	while (1)
	{
		int frame = SimOneAPI::Wait();

		// 如果模拟场景停止，保存评估记录并退出
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		if (!SimOneAPI::GetGps(MainVehicleId, gpsPtr.get()))
		{
			std::cout << "Fetch GPS failed" << std::endl;;
		}

#if AVP_LOG
		AVPLog::getInstance().addData(gpsPtr->posX);
		AVPLog::getInstance().addData(gpsPtr->posY);
#endif // AVP_LOG

		if (status == AVPStatus::eForwarding) // 接近停车位
		{
			if (UtilMath::PlanarDistance({ gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ }, planner.ForwardTrajectory().back()) < 0.05)
			{
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
				status = AVPStatus::eBrakingPrep;
				std::cout << "Reached reverse point!" << std::endl;
			}
			else
			{
				double steering = UtilDriver::CalculateSteering(planner.ForwardTrajectory(), gpsPtr.get());
				UtilDriver::SetControl(gpsPtr->timestamp, 0.05, 0., steering);
			}
		}
		else if (status == AVPStatus::eBrakingPrep) // 停车准备
		{
			if (!brakingPrepStarted)
			{
				startTime = std::chrono::system_clock::now();
				brakingPrepStarted = true;
			}
			else
			{
				// 等待1秒钟
				std::chrono::duration<double> currentBrakingPrepTime = std::chrono::system_clock::now() - startTime;
				if (currentBrakingPrepTime.count() > brakingPrepTime)
				{
					status = AVPStatus::eReversing;
				}
			}
			UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
		}
		else if (status == AVPStatus::eReversing) // 倒车
		{
			if (UtilMath::CloseEnough(std::atan2(targetParkingSpace.heading.y, targetParkingSpace.heading.x),
				(double)gpsPtr->oriZ, UtilUnit::DegreeToRad(headingErrorThresholdDegree)))
			{
				status = AVPStatus::eFinishing;
				std::cout << "Start reversing straightly!" << std::endl;
			}
			else
			{
				double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));
				double throttle, brake;
				if (speedKmH < 3 && !reversingStarted)
				{
					throttle = 0.03f;
					brake = 0.f;
				}
				else
				{
					reversingStarted = true;
					throttle = 0.f;
					brake = 0.02f;
				}
				double steering = UtilDriver::CalculateSteering(planner.ReverseTrajectory(), gpsPtr.get());
				UtilDriver::SetControl(gpsPtr->timestamp, throttle, brake, steering, ESimOne_Gear_Mode_Reverse);
			}
		}
		else if (status == AVPStatus::eFinishing) // 完成停车
		{
			double endDistance = UtilMath::Distance({ gpsPtr->posX, gpsPtr->posY }, planner.ParkingEndPoint());
			if (endDistance < parkingEndDistanceThreshold)
			{
				std::cout << "Finished parking!" << std::endl;
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
				if (leaveAfterParked)
				{
					status = AVPStatus::eWaiting;
				}
				else
				{
					status = AVPStatus::eEnded;
					break;
				}
			}
			// 让车辆滑行
			UtilDriver::SetControl(gpsPtr->timestamp, 0., 0., 0., ESimOne_Gear_Mode_Reverse);
		}
		else if (status == AVPStatus::eWaiting)
		{
			if (!waitingStarted)
			{
				startTime = std::chrono::system_clock::now();
				waitingStarted = true;
			}
			else
			{
				// 等待指定时间
				std::chrono::duration<double> currentWaitingTimeSecond = std::chrono::system_clock::now() - startTime;
				if (currentWaitingTimeSecond.count() > waitingTimeSecond)
				{
					status = AVPStatus::eLeaving;
					std::cout << "Finished waiting for " << waitingTimeSecond << " second!" << std::endl;
				}
			}
			UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
		}
		else if (status == AVPStatus::eLeaving)
		{
			double endDistance = UtilMath::PlanarDistance({ gpsPtr->posX, gpsPtr->posY, 0. }, planner.LeavingTrajectory().back());
			if (endDistance < leavingEndDistanceThreshold)
			{
				status = AVPStatus::eEnded;
				std::cout << "Finished leaving!" << std::endl;
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
			}
			else
			{
				double steering = UtilDriver::CalculateSteering(planner.LeavingTrajectory(), gpsPtr.get());
				UtilDriver::SetControl(gpsPtr->timestamp, 0.05, 0., steering);
			}
		}

		SimOneAPI::NextFrame(frame);
	}

	return 0;
}
