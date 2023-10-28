/*!
* Utility functions for calculation
*
* @file UtilMath.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

#include <cmath>

#include "SSD/SimPoint2D.h"
#include "SSD/SimPoint3D.h"

class UtilMath
{
public:
	/*!
	* @function Sign
	* @brief Compute sign of the variable
	* @param
	*   val: Interested variable
	* @return
	*   1 if positive, -1 if negative, and 0 if 0
	*/
	template <typename T>
	static int Sign(const T val)
	{
		return (val > (T)0) - (val < (T)0);
	}

	/*!
	* @function IsBetween
	* @brief Determine whether a variable is between the other two variables
	* @param
	*   val: Interested variable
	* @param
	*   p1: One of the two boundary points
	* @param
	*   p2: One of the two boundary points
	* @return
	*   True if the given variable is between the other two variables; false otherwise
	*/
	template <typename T>
	static bool IsBetween(const T val, const T p1, const T p2)
	{
		return (p1 < val && val < p2) || (p2 < val && val < p1);
	}

	/*!
	* @function IsBetweenMargin
	* @brief Determine whether a variable is between the other two variables (equality accepted)
	*   val: Interested variable
	* @param
	*   p1: One of the two boundary points
	* @param
	*   p2: One of the two boundary points
	* @return
	* @return
	*   True if the given variable is between the other two variables; false otherwise
	*/
	template <typename T>
	static bool IsBetweenMargin(const T val, const T p1, const T p2)
	{
		return (p1 <= val && val <= p2) || (p2 <= val && val <= p1);
	}

	/*!
	* @function InRectangle
	* @brief Determine whether a point is inside a rectangle (edges excluded)
	* @param
	*   pt: Interested point
	* @param
	*   vertex1: One of the vertices of the rectangle
	* @param
	*   vertex2: One of the vertices of the rectangle (not adjacent to vertex1)
	* @return
	*   True if the point is inside the rectangle; false otherwise
	*/
	static bool InRectangle(const SSD::SimPoint3D& pt, const SSD::SimPoint3D& vertex1, const SSD::SimPoint3D& vertex2)
	{
		return IsBetween(pt.x, vertex1.x, vertex2.x) && IsBetween(pt.y, vertex1.y, vertex2.y);
	}

	/*!
	* @function InRectangleMargin
	* @brief Determine whether a point is inside a rectangle (edges included)
	* @param
	*   pt: Interested point
	* @param
	*   vertex1: One of the vertices of the rectangle
	* @param
	*   vertex2: One of the vertices of the rectangle (not adjacent to vertex1)
	* @return
	*   True if the point is inside the rectangle; false otherwise
	*/
	static bool InRectangleMargin(const SSD::SimPoint3D& pt, const SSD::SimPoint3D& vertex1, const SSD::SimPoint3D& vertex2)
	{
		return IsBetweenMargin(pt.x, vertex1.x, vertex2.x) && IsBetweenMargin(pt.y, vertex1.y, vertex2.y);
	}

	/*!
	* @function CloseEnough
	* @brief Determine wheter the distance between two variables is smaller than the threshold
	* @param
	*   p1: One of the interested variables
	* @param
	*   p2: One of the interested variables
	* @param
	*   threshold: Threshold of the closeness
	* @return
	*   True if the distance between two variables is smaller than the threshold; false otherwise
	*/
	template <typename T>
	static bool CloseEnough(const T p1, const T p2, const T threshold)
	{
		return std::abs(p1 - p2) < threshold;
	}

	/*!
	* @function Length
	* @brief Compute length of the 2D vector
	* @param
	*   vec: Interested 2D vector
	* @return
	*   Length of the 2D vector
	* @overload
	*/
	static double Length(const SSD::SimPoint2D& vec)
	{
		return sqrt(vec.x * vec.x + vec.y * vec.y);
	}

	/*!
	* @function Length
	* @brief Compute length of the 3D vector
	* @param
	*   vec: Interested 3D vector
	* @return
	*   Length of the 3D vector
	* @overload
	*/
	static double Length(const SSD::SimPoint3D& vec)
	{
		return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	}

	/*!
	* @function Dot
	* @brief Compute the dot (inner) product of two 2D vectors
	* @param
	*   v1: One of the interested 2D vectors
	* @param
	*   v2: One of the interested 2D vectors
	* @return
	*   Dot (inner) product of two 2D vectors
	* @overload
	*/
	static double Dot(const SSD::SimPoint2D& v1, const SSD::SimPoint2D v2)
	{
		return v1.x * v2.x + v1.y * v2.y;
	}

	/*!
	* @function Dot
	* @brief Compute the dot (inner) product of two 3D vectors
	* @param
	*   v1: One of the interested 3D vectors
	* @param
	*   v2: One of the interested 3D vectors
	* @return
	*   Dot (inner) product of two 3D vectors
	* @overload
	*/
	static double Dot(const SSD::SimPoint3D& v1, const SSD::SimPoint3D v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

	/*!
	* @function Angle
	* @brief Compute the angle between two 2D vectors
	* @param
	*   v1: One of the interested 2D vectors
	* @param
	*   v2: One of the interested 2D vectors
	* @return
	*   Angle between two 2D vectors in radians
	*/
	static double Angle(const SSD::SimPoint2D& v1, const SSD::SimPoint2D& v2)
	{
		double l1 = Length(v1);
		double l2 = Length(v2);
		double dot = Dot(v1, v2);
		return acos(Dot(v1, v2) / (Length(v1) * Length(v2)));
	}

	/*!
	* @function Distance
	* @brief Compute the distance between two 2D points
	* @param
	*   pt1: One of the interested 2D points
	* @param
	*   pt2: One of the interested 2D points
	* @return
	*   Distance between two 2D points
	* @overload
	*/
	static double Distance(const SSD::SimPoint2D& pt1, const SSD::SimPoint2D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
	}

	/*!
	* @function Distance
	* @brief Compute the distance between two 3D points
	* @param
	*   pt1: One of the interested 3D points
	* @param
	*   pt2: One of the interested 3D points
	* @return
	*   Distance between two 3D points
	* @overload
	*/
	static double Distance(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2) + std::pow(pt1.z - pt2.z, 2));
	}

	/*!
	* @function PlanarDistance
	* @brief Compute the planar distance between two 3D points
	* @param
	*   pt1: One of the interested 3D points
	* @param
	*   pt2: One of the interested 3D points
	* @return
	*   Planar distance between two 3D points
	*/
	static double PlanarDistance(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
	{
		return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
	}

	/*!
	* @function Rotate
	* @brief Rotate a point for a certain angle
	* @param
	*   ori: Interested point
	* @param
	*   theta: Angle to rotate in radians
	@return
	*   Rotated point
	*/
	static SSD::SimPoint2D Rotate(const SSD::SimPoint2D& ori, const double theta)
	{
		SSD::SimPoint2D res;
		res.x = ori.x * cos(theta) + ori.y * sin(theta);
		res.y = -ori.x * sin(theta) + ori.y * cos(theta);
		return std::move(res);
	}

	/*!
	* @function Translate
	* @brief Translate a point with a certain vector
	* @param
	*   ori: Interested point
	* @param
	*   dir: Vector to translate
	@return
	*   Translated point
	*/
	static SSD::SimPoint2D Translate(const SSD::SimPoint2D& ori, const SSD::SimPoint2D& dir)
	{
		SSD::SimPoint2D res;
		res.x = ori.x + dir.x;
		res.y = ori.y + dir.y;
		return std::move(res);
	}

	/*!
	* @function LocalToGlobal
	* @brief Transform a point in local coordinate to global coordinate
	* @param
	*   originCoord: Coordinate of the origin of the local coordinate in global coordinate
	* @param
	*   theta: Rotation angle of the local coordinate with respect to the global coordinate
	* @param
	*   ptLocal: coordinate of the interested point in local coordinate
	@return
	*   Transformed point in global coordinate
	*/
	static SSD::SimPoint2D LocalToGlobal(const SSD::SimPoint2D& originCoord, const double theta, const SSD::SimPoint2D& ptLocal)
	{
		SSD::SimPoint2D rotatedPt = Rotate(ptLocal, -theta);
		SSD::SimPoint2D res = Translate(rotatedPt, { originCoord.x, originCoord.y });
		return std::move(res);
	}
};
