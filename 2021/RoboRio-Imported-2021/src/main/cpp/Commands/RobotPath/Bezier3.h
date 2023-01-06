//==============================================================================
// Bezier3.h
//==============================================================================
#pragma once

#include "Point2D.h"

// A 2d cubic Bezier curve segment
class Bezier3
{
public:
	//==========================================================================
	// Properties

	Point2D m_point1;		// First point on the control polygon
	Point2D m_point2;		// Second point on the control polygon
	Point2D m_point3;		// Third point on the control polygon
	Point2D m_point4;		// Forth point on the control polygon


	//==========================================================================
	// Construction

	// Default constructor
	Bezier3();


	//==========================================================================
	// Operations

	// Evaluate the curve at the given parameter value
	//
	// u -  Parameter value to evaluate at. Range is [0.0, 1.0]
	//
	// Returns the point on the curve
	Point2D Evaluate(double u) const;

	// Evaluate the derivate of the curve at the given parameter value
	//
	// u - Parameter value to evaluate at. Range is [0.0, 1.0]</param>
	//
	// Returns the derivative of the curve
	Point2D Derivative(double u) const;

	// Evaluate the second derivate of the curve at the given parameter value
	//
	// u - Parameter value to evaluate at. Range is [0.0, 1.0]</param>
	//
	// Returns the second derivative of the curve
	Point2D Derivative2(double u) const;

	// Evaluate the signed curvature of the curve at the given parameter value
	//
	// u - Parameter value to evaluate at. Range is [0.0, 1.0]</param>
	//
	// Returns the signed curvature of the curve, with +ve to the left and -ve to
    // the right (when travelling in the direction of increasing u)
	double Curvature(double u) const;

	// Split the curve into two at a given parameter position
	//
	// u -  Parameter value to split at. Range is [0.0, 1.0]
	// bezier3_1 - Returns the first part of the curve (i.e. before the split)
	// bezier3_2 - Returns the second part of the curve (i.e. after the split)
	void Split(double u, Bezier3& bezier3_1, Bezier3& bezier3_2) const;

	// Calculate the length of the curve using subdivision
	//
	// error_ratio - Error ratio to stop the subdivision at (smaller is more precise, but slower)
	// max_depth - Maximum depth of subdivision (larger may be more precise and slower)
	//
	// Returns the length of the curve
	double Length(double error_ratio = 0.01, int max_depth = 5) const;

	// Calculate the position of a point a given arc distance along the curve
	//
	// arc_distance - Distance along the curve to calculate the point at
	// u - Returns the curve parameter value at the calculated point. This may be a poor approximatation but will
	// provide a position where a good approximation of the derivative can be obtained.
	// error_ratio - Error ratio to stop the subdivision at when calculating curve distance (smaller is more precise, but slower)
	// max_depth - Maximum depth of subdivision when calculating curve distance (larger may be more precise and slower)
	//
	// Returns the point on the curve
	Point2D ArcLengthPoint(double arc_distance, double& u, double error_ratio = 0.01, int max_depth = 5) const;
private:

	// Search for the point and parameter value of the point that is a given arc distance along the curve,
	// which may lie beyond the end of the curve
	//
	// arc_distance - Distance along the curve to find the point at
	// result - Returns the point on the curve, if successful.
	// u - Returns the curve parameter value at the calculated point, if successful. This may be a poor approximatation but will
	// provide a position where a good approximation of the derivative can be obtained.
	// total_length - Returns the total length of the curve
	// error_ratio - Error ratio to stop the subdivision at when calculating curve distance (smaller is more precise, but slower)
	// max_depth - Maximum depth of subdivision when calculating curve distance (larger may be more precise and slower)
	//
	// Returns whether the point was found on this curve
	bool ArcLengthSearch(double arc_distance, Point2D& result, double& u, double& total_length, double error_ratio, int max_depth) const;
};

