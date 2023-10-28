/*!
* Parameters of the vehicle
*
* @file VehicleParam.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

struct VehicleParam
{
	//@brief Distance from back of the vehicle to center of its rear axis
	double Lr;

	//@brief Distance from front of the vehicle to center of its front axis
	double Lf;

	//@brief Distance between two axis
	double L;
};
