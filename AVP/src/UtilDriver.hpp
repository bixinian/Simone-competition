/*!
* Utility functions for vehicle control
*
* @file UtilDriver.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <algorithm>

#include "SimOneServiceAPI.h"

class UtilDriver
{
public:
	/*!
	* @function SetControl
	* @brief Set control of a main vehicle
	* @param
	*   timestamp: Timestamp of the control signal
	* @param
	*   throttle: Throttle to be set
	* @param
	*   brake: Brake to be set
	* @param
	*   steering: Steering to be set
	* @param
	*   gear: Gear to be set, defaulted with ESimOne_Gear_Mode_Drive
	* @param
	*   mainVehicleId: Id of the controlled main vehicle
	*/
	static void SetControl(const long long timestamp, const double throttle, const double brake,
						const double steering, const ESimOne_Gear_Mode gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive,
						const char* mainVehicleId = "0")
	{
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique <SimOne_Data_Control>();

		pControl->timestamp = timestamp;
		pControl->throttle = (float)throttle;
		pControl->brake = (float)brake;
		pControl->steering = (float)steering;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = gear;
		SimOneAPI::SetDrive(mainVehicleId, pControl.get());
	}
	
	/*!
	* @function CalculateSteering
	* @brief Compute steering of the vehicle (for control purposes)
	* @param
	*   targetPath: Planned trajectory
	* @param
	*   pGps: Pointer of current gps signal
	* @return
	*   Steering of the vehicle
	*/
	static double CalculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps)
	{
		std::vector<double> pts;
		for (size_t i = 0; i < targetPath.size(); ++i)
		{
			pts.push_back(pow(pGps->posX - targetPath[i].x, 2.) + pow(pGps->posY - targetPath[i].y, 2.));
		}

		size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();

		size_t forwardIndex = 0;
		double minProgDist = 3.;
		double progTime = 0.8;
		double mainVehicleSpeed = (double)sqrtf(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		for (; index < targetPath.size(); ++index)
		{
			forwardIndex = index;
			double distance = sqrt(pow(targetPath[index].x - pGps->posX, 2.) + pow(targetPath[index].y - pGps->posY, 2.));
			if (distance >= progDist)
			{
				break;
			}
		}

		double psi = (double)pGps->oriZ;
		double Alfa = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
		double ld = sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2.) + pow(targetPath[forwardIndex].x - pGps->posX, 2.));
		double steering = -atan2(2. * (1.3 + 1.55) * sin(Alfa), ld) * 36. / (7. * M_PI);
		return steering;
	}
};