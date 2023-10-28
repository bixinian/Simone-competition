/*!
* Planner for the AVP sample (perpendicular parking)
*
* @file AVPPanner.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

#include "VehicleParam.hpp"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"

class AVPPlanner
{
public:
	/*!
	* @function AVPPlanner
	* @brief Construct an instance of AVPPlanner
	* @param 
	*   parkingSpace: Target parking space
	* @param
	*   initPoint: Starting point of the planning
	* @param
	*   terminalPoint: Terminal point of the planning for leaving
	* @param
	*   veh: Parameters of the vehicle
	* @param
	*   leaveAfterParked: Whether leaving after finished parking is needed
	* @param
	*   safetyDistance: Safety distance of the rear of the vehicle to the boundary of the parking space
	*/
	AVPPlanner(const HDMapStandalone::MParkingSpace& parkingSpace,
		const SSD::SimPoint3D& initPoint,
		const SSD::SimPoint3D& terminalPoint,
		const VehicleParam& veh,
		const bool leaveAfterParked = false,
		const double safetyDistance = 0.);
	AVPPlanner() = delete;
	~AVPPlanner() = default;

	void plan();
	const SSD::SimPoint3DVector ReverseTrajectory() const;
	const SSD::SimPoint3DVector ForwardTrajectory() const;
	const SSD::SimPoint2D ParkingEndPoint() const;
	const SSD::SimPoint3DVector LeavingTrajectory() const;

private:
	void planReverseTrajectory();
	void planForwardTrajectory();
	void planLeavingTrajectory();

	//@brief Target parking space
	HDMapStandalone::MParkingSpace mParkingSpace;

	//@brief Starting point of the planning
	SSD::SimPoint3D mInitPoint;

	//@brief Terminal point of the planning for leaving
	SSD::SimPoint3D mTerminalPoint;

	//@brief Parameters of the vehicle
	VehicleParam mVeh;

	//@brief Whether leaving after finished parking is required
	bool mLeaveAfterParked;

	//@brief Safety distance of the rear of the vehicle to the
	// boundary of the parking space
	double mSafetyDistance;

	// origin of the parking space in global frame
	SSD::SimPoint2D mOrigin;

	//@brief x axis of the parking space in global frame
	SSD::SimPoint2D mAxis;

	//@brief Orientation of the parking space in global frame
	double mOrientation;

	//@brief The point from which the vehicle starts to reverse in 
	// parking space frame (whose origin is its top left knot)
	SSD::SimPoint2D mReversePointLocal;

	//@brief The point from which the vehicle starts to reverse in global frame
	SSD::SimPoint2D mReversePoint;

	//@brief The point where the vehicle stops in global frame
	SSD::SimPoint2D mParkingEndPoint;

	//@brief Move the control point by this value to compensate
	// for potential understeering
	double mTurningCompensation;

	//@brief Step size of the reversing trajectory
	double mReverseTrajectoryStepSize;

	//@brief Planned reversing trajectory
	// from mReversePoint to the parking space
	SSD::SimPoint3DVector mReverseTrajectory;

	//@brief Step size of the forward trajectory
	double mForwardTrajectoryStepSize;

	//@brief Planned forward trajectory
	// from mInitPoint to mReversePoint
	SSD::SimPoint3DVector mForwardTrajectory;

	//@brief Step size of the leaving trajectory
	double mLeavingTrajectoryStepSize;

	//@brief Planned leaving trajectory
	// from mParkingEndPoint to mTerminalPoint
	SSD::SimPoint3DVector mLeavingTrajectory;
};
