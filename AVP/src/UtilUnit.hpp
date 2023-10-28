/*!
* Utility functions for unit conversion
*
* @file UtilUnit.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

class UtilUnit
{
public:
	/*!
	* @function RadToDegree
	* @brief convert rads to degrees
	* @param
	*   rad: Angle in radians
	* @return
	*   Angle in degrees
	*/
	static double RadToDegree(const double rad)
	{
		return rad * 180. / M_PI;
	}

	/*!
	* @function DegreeToRad
	* @brief convert degrees to radians
	* @param
	*   degree: Angle in degrees
	* @return
	*   Angle in radians
	*/
	static double DegreeToRad(const double degree)
	{
		return degree / 180. * M_PI;
	}

	/*!
	* @function MsToKmH
	* @brief convert meters per second to kilometers per hour
	* @param
	*   ms: speed in m/s
	* @return
	*   speed in km/h
	*/
	static double MsToKmH(const double ms)
	{
		return ms * 3.6;
	}

	/*!
	* @function KmHToMs
	* @brief convert kilometers per hour to meters per second
	* @param
	*   kmh: speed in km/h
	* @return
	*   speed in m/s
	*/
	static double KmHToMs(const double kmh)
	{
		return kmh / 3.6;
	}
};
