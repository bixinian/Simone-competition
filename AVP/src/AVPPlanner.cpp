#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Eigen/Core"
#include "Eigen/QR"
#include "UtilMath.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"

#if AVP_LOG
// ��AVP_LOG����Ϊ��ʱ�����ڼ�¼��־�ĸ�������

// ��¼��ά������
void logPoint(const SSD::SimPoint2D& pt)
{
	AVPLog::getInstance().addData(pt.x);
	AVPLog::getInstance().addData(pt.y);
}

// ��¼��ά������
void logPoint(const SSD::SimPoint3D& pt)
{
	AVPLog::getInstance().addData(pt.x);
	AVPLog::getInstance().addData(pt.y);
}

// ��¼���з�
void logNewLine()
{
	AVPLog::getInstance().addNewLine();
}
#endif // AVP_LOG

// ���캯������ʼ��AVPPlanner����
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
	// ��ʼ���ֲ�����ϵԭ�������
	mOrigin.x = mParkingSpace.boundaryKnots[0].x;
	mOrigin.y = mParkingSpace.boundaryKnots[0].y;
	mAxis.x = mParkingSpace.boundaryKnots[3].x - mParkingSpace.boundaryKnots[0].x;
	mAxis.y = mParkingSpace.boundaryKnots[3].y - mParkingSpace.boundaryKnots[0].y;

	// ����ֲ�����ϵ����
	mOrientation = std::atan2(mAxis.y, mAxis.x);
}

// ��ȡ����켣
const SSD::SimPoint3DVector AVPPlanner::ReverseTrajectory() const
{
	return mReverseTrajectory;
}

// ��ȡǰ��켣
const SSD::SimPoint3DVector AVPPlanner::ForwardTrajectory() const
{
	return mForwardTrajectory;
}

// ��ȡͣ���յ�
const SSD::SimPoint2D AVPPlanner::ParkingEndPoint() const
{
	return mParkingEndPoint;
}

// ��ȡ�뿪�켣
const SSD::SimPoint3DVector AVPPlanner::LeavingTrajectory() const
{
	return mLeavingTrajectory;
}

// �滮����켣
void AVPPlanner::planReverseTrajectory()
{
	/* 1. ������Ƶ� */
	// ͣ���ռ�ĳ��ȺͿ��
	double pL = UtilMath::PlanarDistance(mParkingSpace.boundaryKnots[0], mParkingSpace.boundaryKnots[3]);
	double pW = UtilMath::PlanarDistance(mParkingSpace.boundaryKnots[0], mParkingSpace.boundaryKnots[1]);

	// ���ѡ����ʼ��C0
	double x0_ori = mReversePointLocal.x;
	double y0_ori = mReversePointLocal.y;
	// ������ϵ��תpi/4�Է������
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

	// �����յ�
	double xe_ori = pL / 2.;
	double vehicleLength = mVeh.Lf + mVeh.L + mVeh.Lr;
	double ye_ori = -pW + mVeh.Lr + (pW - vehicleLength) / 2.;
	SSD::SimPoint2D Ce = UtilMath::Rotate({ xe_ori, ye_ori }, M_PI_4);
	mParkingEndPoint = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, Ce);

#if AVP_LOG
	// ��¼���Ƶ�
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


	/* 2. ʹ��Eigen��Ϸ���켣 */
	Eigen::MatrixXd A(5, 5);
	A << pow(x0, 4), pow(x0, 3), pow(x0, 2), pow(x0, 1), 1,
		pow(xc1, 4), pow(xc1, 3), pow(xc1, 2), pow(xc1, 1), 1,
		pow(xc3, 4), pow(xc3, 3), pow(xc3, 2), pow(xc3, 1), 1,
		4 * pow(xc3, 3), 3 * pow(xc3, 2), 2 * pow(xc3, 1), 1, 0,
		pow(xc2, 4), pow(xc2, 3), pow(xc2, 2), pow(xc2, 1), 1;
	Eigen::VectorXd b(5);
	b << y0, yc1, yc3, kc3, yc2;
	auto xCoeff = A.householderQr().solve(b);

	// ������ϵ�������ʣ��������¼��㣬ֱ���õ����ʵ�ϵ��
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

	// ���㷴��켣�ľֲ������
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

	/* 3. ���ֲ��켣ת��Ϊȫ�ֹ켣 */
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

// �滮ǰ��켣
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

// �滮�뿪�켣
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

// �滮�����켣
void AVPPlanner::plan()
{
	// �滮����켣
	planReverseTrajectory();
	// �滮ǰ��켣
	planForwardTrajectory();
	if (mLeaveAfterParked)
	{
		// �����Ҫ��ͣ�����뿪����滮�뿪�켣
		planLeavingTrajectory();
	}
#if AVP_LOG
	else
	{
		// ��¼����켣�ĵ�һ����
		logPoint(mReverseTrajectory.front());
		logNewLine();
	}
#endif // AVP_LOG
}
