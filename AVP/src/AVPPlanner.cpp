#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Eigen/Core"
#include "Eigen/QR"
#include "UtilMath.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"

#if AVP_LOG
// 在AVP_LOG定义为真时，用于记录日志的辅助函数

// 记录二维点坐标
void logPoint(const SSD::SimPoint2D& pt)
{
	AVPLog::getInstance().addData(pt.x);
	AVPLog::getInstance().addData(pt.y);
}

// 记录三维点坐标
void logPoint(const SSD::SimPoint3D& pt)
{
	AVPLog::getInstance().addData(pt.x);
	AVPLog::getInstance().addData(pt.y);
}

// 记录换行符
void logNewLine()
{
	AVPLog::getInstance().addNewLine();
}
#endif // AVP_LOG

// 构造函数：初始化AVPPlanner对象
AVPPlanner::AVPPlanner(const HDMapStandalone::MParkingSpace& parkingSpace,
	const SSD::SimPoint3D& initPoint,
	const SSD::SimPoint3D& terminalPoint,
	const VehicleParam& veh,
	const bool leaveAfterParked,
	const double safetyDistance) :
	mParkingSpace(parkingSpace),
	mInitPoint(initPoint),
	mTerminalPoint(terminalPoint),
	mVeh(veh),
	mLeaveAfterParked(leaveAfterParked),
	mSafetyDistance(safetyDistance),
	mReversePointLocal({ 7., 4. }),
	mReversePoint(),
	mParkingEndPoint(),
	mTurningCompensation(0.4),
	mReverseTrajectoryStepSize(0.2),
	mReverseTrajectory(),
	mForwardTrajectoryStepSize(0.5),
	mForwardTrajectory(),
	mLeavingTrajectoryStepSize(0.5),
	mLeavingTrajectory()
{
	// 初始化局部坐标系原点和轴向
	mOrigin.x = mParkingSpace.boundaryKnots[0].x;
	mOrigin.y = mParkingSpace.boundaryKnots[0].y;
	mAxis.x = mParkingSpace.boundaryKnots[3].x - mParkingSpace.boundaryKnots[0].x;
	mAxis.y = mParkingSpace.boundaryKnots[3].y - mParkingSpace.boundaryKnots[0].y;

	// 计算局部坐标系方向
	mOrientation = std::atan2(mAxis.y, mAxis.x);
}

// 获取反向轨迹
const SSD::SimPoint3DVector AVPPlanner::ReverseTrajectory() const
{
	return mReverseTrajectory;
}

// 获取前向轨迹
const SSD::SimPoint3DVector AVPPlanner::ForwardTrajectory() const
{
	return mForwardTrajectory;
}

// 获取停车终点
const SSD::SimPoint2D AVPPlanner::ParkingEndPoint() const
{
	return mParkingEndPoint;
}

// 获取离开轨迹
const SSD::SimPoint3DVector AVPPlanner::LeavingTrajectory() const
{
	return mLeavingTrajectory;
}

// 规划反向轨迹
void AVPPlanner::planReverseTrajectory()
{
	/* 1. 计算控制点 */
	// 停车空间的长度和宽度
	double pL = UtilMath::PlanarDistance(mParkingSpace.boundaryKnots[0], mParkingSpace.boundaryKnots[3]);
	double pW = UtilMath::PlanarDistance(mParkingSpace.boundaryKnots[0], mParkingSpace.boundaryKnots[1]);

	// 随机选择起始点C0
	double x0_ori = mReversePointLocal.x;
	double y0_ori = mReversePointLocal.y;
	// 将坐标系旋转pi/4以方便计算
	SSD::SimPoint2D C0 = UtilMath::Rotate({ x0_ori, y0_ori }, M_PI_4);
	mReversePoint = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C0);
	double x0 = C0.x;
	double y0 = C0.y;
	double kc0 = -1.;
	double xc2_ori = pL / 2 + mTurningCompensation;
	double yc2_ori = -pW + (mSafetyDistance + mVeh.Lr) + mVeh.L;
	SSD::SimPoint2D C2 = UtilMath::Rotate({ xc2_ori, yc2_ori }, M_PI_4);
	double xc2 = C2.x;
	double yc2 = C2.y;
	double xc3_ori = xc2_ori;
	double yc3_ori = -pW + (mSafetyDistance + mVeh.Lr);
	SSD::SimPoint2D C3 = UtilMath::Rotate({ xc3_ori, yc3_ori }, M_PI_4);
	double xc3 = C3.x;
	double yc3 = C3.y;
	double kc3 = 1.;

	double R = y0_ori - yc2_ori;
	double xc1_ori = xc2_ori + R;
	double yc1_ori = y0_ori;
	SSD::SimPoint2D C1 = UtilMath::Rotate({ xc1_ori, yc1_ori }, M_PI_4);
	double xc1 = C1.x;
	double yc1 = C1.y;

	// 计算终点
	double xe_ori = pL / 2.;
	double vehicleLength = mVeh.Lf + mVeh.L + mVeh.Lr;
	double ye_ori = -pW + mVeh.Lr + (pW - vehicleLength) / 2.;
	SSD::SimPoint2D Ce = UtilMath::Rotate({ xe_ori, ye_ori }, M_PI_4);
	mParkingEndPoint = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, Ce);

#if AVP_LOG
	// 记录控制点
	SSD::SimPoint2D C0_Global = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C0);
	logPoint(C0_Global);
	SSD::SimPoint2D C1_Global = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C1);
	logPoint(C1_Global);
	SSD::SimPoint2D C2_Global = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C2);
	logPoint(C2_Global);
	SSD::SimPoint2D C3_Global = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C3);
	logPoint(C3_Global);
	logNewLine();
#endif // AVP_LOG


	/* 2. 使用Eigen拟合反向轨迹 */
	Eigen::MatrixXd A(5, 5);
	A << pow(x0, 4), pow(x0, 3), pow(x0, 2), pow(x0, 1), 1,
		pow(xc1, 4), pow(xc1, 3), pow(xc1, 2), pow(xc1, 1), 1,
		pow(xc3, 4), pow(xc3, 3), pow(xc3, 2), pow(xc3, 1), 1,
		4 * pow(xc3, 3), 3 * pow(xc3, 2), 2 * pow(xc3, 1), 1, 0,
		pow(xc2, 4), pow(xc2, 3), pow(xc2, 2), pow(xc2, 1), 1;
	Eigen::VectorXd b(5);
	b << y0, yc1, yc3, kc3, yc2;
	auto xCoeff = A.householderQr().solve(b);

	// 如果拟合系数不合适，则多次重新计算，直到得到合适的系数
	std::vector<double> coeffs{ xCoeff[4], xCoeff[3], xCoeff[2], xCoeff[1], xCoeff[0] };
	while (std::abs(coeffs[0]) > 5 || isnan(coeffs[0])
		|| std::abs(coeffs[1]) > 5 || isnan(coeffs[1])
		|| std::abs(coeffs[2]) > 5 || isnan(coeffs[2])
		|| std::abs(coeffs[3]) > 5 || isnan(coeffs[3])
		|| std::abs(coeffs[4]) > 5 || isnan(coeffs[4]))
	{
		auto xCoeffNew = A.householderQr().solve(b);
		coeffs[0] = xCoeffNew[4];
		coeffs[1] = xCoeffNew[3];
		coeffs[2] = xCoeffNew[2];
		coeffs[3] = xCoeffNew[1];
		coeffs[4] = xCoeffNew[0];
	}
	std::cout << "Fitted coefficients: [" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2]
		<< ", " << coeffs[3] << ", " << coeffs[4] << "]" << std::endl;
	std::cout << std::endl;

	// 计算反向轨迹的局部坐标点
	std::vector<SSD::SimPoint2D> reverseTrajLocal;
	double reverseStepSize = mReverseTrajectoryStepSize * (xc3 - x0) / abs(xc3 - x0);
	int reverseStepCount = (int)((xc3 - x0) / reverseStepSize);
	for (size_t i = 0; i <= reverseStepCount; ++i)
	{
		double x = x0 + i * reverseStepSize;
		double y = coeffs[4] * pow(x, 4) + coeffs[3] * pow(x, 3) + coeffs[2] * pow(x, 2)
			+ coeffs[1] * pow(x, 1) + coeffs[0];
		reverseTrajLocal.push_back({ x, y });
	}
	reverseTrajLocal.push_back({ xc3, yc3 });

	/* 3. 将局部轨迹转换为全局轨迹 */
	mReverseTrajectory.resize(reverseTrajLocal.size());
	for (size_t i = 0; i < reverseTrajLocal.size(); ++i)
	{
		SSD::SimPoint2D trajPt = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, reverseTrajLocal[i]);
		mReverseTrajectory[i] = { trajPt.x, trajPt.y, 0. };
#if AVP_LOG
		logPoint(mReverseTrajectory[i]);
#endif // AVP_LOG
	}

#if AVP_LOG
	logNewLine();
#endif // AVP_LOG
}

// 规划前向轨迹
void AVPPlanner::planForwardTrajectory()
{
	double xDiff = mReversePoint.x - mInitPoint.x;
	double yDiff = mReversePoint.y - mInitPoint.y;
	if (std::abs(xDiff) > std::abs(yDiff)) {
		double forwardStepSize = mForwardTrajectoryStepSize * UtilMath::Sign(xDiff);
		int forwardStepCount = (int)(xDiff / forwardStepSize);
		for (size_t i = 0; i <= forwardStepCount; ++i)
		{
			double x = mInitPoint.x + i * forwardStepSize;
			double y = mInitPoint.y + i * forwardStepSize / xDiff * yDiff;
			mForwardTrajectory.push_back({ x, y, 0. });
#if AVP_LOG
			logPoint(mForwardTrajectory.back());
#endif // AVP_LOG
		}

#if AVP_LOG
		logNewLine();
#endif // AVP_LOG
	}
	else {
		double forwardStepSize = mForwardTrajectoryStepSize * UtilMath::Sign(yDiff);
		int forwardStepCount = (int)(yDiff / forwardStepSize);
		for (size_t i = 0; i <= forwardStepCount; ++i)
		{
			double y = mInitPoint.y + i * forwardStepSize;
			double x = mInitPoint.x + i * forwardStepSize / yDiff * xDiff;
			mForwardTrajectory.push_back({ x, y, 0. });
#if AVP_LOG
			logPoint(mForwardTrajectory.back());
#endif // AVP_LOG
		}

#if AVP_LOG
		logNewLine();
#endif // AVP_LOG
	}
}

// 规划离开轨迹
void AVPPlanner::planLeavingTrajectory()
{
	size_t reverseTrajSize = mReverseTrajectory.size();
	mLeavingTrajectory.resize(reverseTrajSize);
	for (size_t i = 0; i < reverseTrajSize; ++i) {
		mLeavingTrajectory[i] = mReverseTrajectory[reverseTrajSize - i - 1];
	}

	double xDiff = mReversePoint.x - mInitPoint.x;
	double yDiff = mReversePoint.y - mInitPoint.y;
	if (std::abs(xDiff) > std::abs(yDiff)) {
		double leavingStepSize = mLeavingTrajectoryStepSize * UtilMath::Sign(xDiff);
		int leavingStepCount = (int)(xDiff / leavingStepSize);
		for (size_t i = 0; i <= leavingStepCount; ++i)
		{
			double x = mReversePoint.x + i * leavingStepSize;
			double y = mReversePoint.y + i * leavingStepSize / xDiff * yDiff;
			mLeavingTrajectory.push_back({ x, y, 0. });
#if AVP_LOG
			logPoint(mLeavingTrajectory.back());
#endif // AVP_LOG
		}

#if AVP_LOG
		logNewLine();
#endif // AVP_LOG
	}
	else {
		double leavingStepSize = mLeavingTrajectoryStepSize * UtilMath::Sign(yDiff);
		int leavingStepCount = (int)(yDiff / leavingStepSize);
		for (size_t i = 0; i <= leavingStepCount; ++i)
		{
			double y = mReversePoint.y + i * leavingStepSize;
			double x = mReversePoint.x + i * leavingStepSize / yDiff * xDiff;
			mLeavingTrajectory.push_back({ x, y, 0. });
#if AVP_LOG
			logPoint(mLeavingTrajectory.back());
#endif // AVP_LOG
		}

#if AVP_LOG
		logNewLine();
#endif // AVP_LOG

	}
}

// 规划整个轨迹
void AVPPlanner::plan()
{
	// 规划反向轨迹
	planReverseTrajectory();
	// 规划前向轨迹
	planForwardTrajectory();
	if (mLeaveAfterParked)
	{
		// 如果需要在停车后离开，则规划离开轨迹
		planLeavingTrajectory();
	}
#if AVP_LOG
	else
	{
		// 记录反向轨迹的第一个点
		logPoint(mReverseTrajectory.front());
		logNewLine();
	}
#endif // AVP_LOG
}
