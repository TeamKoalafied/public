//==============================================================================
// Point2D.h
//==============================================================================

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

// 2D point
//
// This class can be use to represent 2D points and vectors
class Point2D
{
public:
	//=============================================================================
	// Member Variables

	// Point coordinates
	double x;
	double y;


	//=============================================================================
	// Construction

	// Default constructor
	Point2D() :
		x(0.0),
		y(0.0) {
	}

	// Value constructor
	Point2D(double _x, double _y) :
		x(_x),
		y(_y) {
	}


	//=============================================================================
	// Operators

	// Set the point
	void Set(double _x, double _y) {
		x = _x;
		y = _y;
	}

	// Unary minus
	Point2D operator-() const {
		return Point2D(-x, -y);
	}

	// Vector addition
	Point2D operator+(const Point2D& pt) const {
		return Point2D(x + pt.x, y + pt.y);
	}

	// Vector subtraction
	Point2D operator-(const Point2D& pt) const {
		return Point2D(x - pt.x, y - pt.y);
	}

	// Scalar multiplication
	Point2D operator*(double scale) const {
		return Point2D(x * scale, y * scale);
	}

	// Scalar multiplication and assignment
	Point2D& operator*=(double scale) {
		x *= scale;
		y *= scale;
		return *this;
	}
	
    // Dot product (aka inner product)
    double Dot(const Point2D pt) {
        return x*pt.x + y*pt.y;
    }
    

	//=============================================================================
	// Angle and Length

	// Get the length of the point as a vector (aka distance from origin)
	double Length() const {
		return sqrt(x*x + y * y);
	}

	// Scale the point so that its length is 1.0. If the point is the origin nothing happens.
	void Normalize() {
		double length = Length();
		if (length != 0.0) {
			x /= length;
			y /= length;
		}
	}

	// Get the angle of a given vector in radians with range [-PI, PI], using the standard
	// mathematical convention (0 is along the x axis, +ve anti-clockwise)
	double AngleRadians() {
		return atan2(y, x);
	}

	// Get the angle of a given vector in degrees with range [-180, 180], using the standard
	// mathematical convention (0 is along the x axis, +ve anti-clockwise)
	double AngleDegrees() {
		return atan2(y, x) * 180.0 / M_PI;
	}

	// Get a unit vector at a given angle in degrees
	static Point2D UnitVectorDegrees(double angle_degrees) {
		double angle_radians = angle_degrees * M_PI / 180.0;
		return Point2D(cos(angle_radians), sin(angle_radians));
	}


	//=============================================================================
	// Static Operations

	// Perform a linear interpolation between 2 points
	//
	// pt1 - Start point. Corresponds to u == 0.0
	// pt2 - End point. Corresponds to u == 1.0
	// u - Interpolation parameter. Range is [0.0, 1.0] for interpolation
	//
	// Returns the inerpolated point
	static Point2D Lerp(const Point2D& pt1, const Point2D& pt2, double u) {
		return Point2D(pt1.x * (1.0 - u) + pt2.x * u, pt1.y * (1.0 - u) + pt2.y * u);
	}
};


//=============================================================================
// Extra Operations
//
// These are needed because in C++ the first argument of an operator is the one
// that the compiler looks at. For example, we can scale a point by multiplying
// by a number
//
//	Point2D(2.0, 3.0) * 10.0;
//	10.0 * Point2D(2.0, 3.0);
//
// The first one will call the member function, but to make the second one work we
// need the global function below. 

inline Point2D operator*(double scale, const Point2D& pt) {
	return Point2D(pt.x * scale, pt.y * scale);
}
