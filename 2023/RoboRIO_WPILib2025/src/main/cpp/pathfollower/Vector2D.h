//==============================================================================
// Vector2D.h
//==============================================================================

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <units/math.h>



// 2D point
//
// This class can be use to represent 2D points and vectors
template <class T>
class Vector2D
{
public:
	//=============================================================================
	// Member Variables

	// Point coordinates
	T x;
	T y;


	//=============================================================================
	// Construction

	// Default constructor
	Vector2D() :
		x(0.0),
		y(0.0) {
	}

	// Value constructor
	Vector2D(T _x, T _y) :
		x(_x),
		y(_y) {
	}


	//=============================================================================
	// Operators

	// Set the point
	void Set(T _x, T _y) {
		x = _x;
		y = _y;
	}

	// Unary minus
	Vector2D operator-() const {
		return Vector2D(-x, -y);
	}

	// Vector addition
	Vector2D operator+(const Vector2D& pt) const {
		return Vector2D(x + pt.x, y + pt.y);
	}

	// Vector subtraction
	Vector2D operator-(const Vector2D& pt) const {
		return Vector2D(x - pt.x, y - pt.y);
	}

	// Scalar multiplication
	Vector2D operator*(double scale) const {
		return Vector2D(x * scale, y * scale);
	}
 

	// Scalar multiplication and assignment
	Vector2D& operator*=(double scale) {
		x *= scale;
		y *= scale;
		return *this;
	}
	
    // Dot product (aka inner product)
    double Dot(const Vector2D pt) {
        return x*pt.x + y*pt.y;
    }
    

	//=============================================================================
	// Angle and Length

	// Get the length of the point as a vector (aka distance from origin)
	T Length() const {
		return units::math::sqrt(x * x + y * y);
	}

	// Scale the point so that its length is 1.0. If the point is the origin nothing happens.
	T Normalize() {
		T length = Length();
		if ((double)length != 0.0) {
			x /= (double)length;
			y /= (double)length;
		}
        return length;
	}

	// Get the angle of a given vector in radians with range [-PI, PI], using the standard
	// mathematical convention (0 is along the x axis, +ve anti-clockwise)
	units::radian_t Angle() {
		return units::radian_t(atan2((double)y, (double)x));
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
	static Vector2D UnitVectorDegrees(double angle_degrees) {
		double angle_radians = angle_degrees * M_PI / 180.0;
		return Vector2D(cos(angle_radians), sin(angle_radians));
	}

	// Get a unit vector at a given angle in degrees
	static Vector2D UnitVector(units::radian_t angle) {
		return Vector2D((T)cos(angle.value()), (T)sin(angle.value()));
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
	static Vector2D Lerp(const Vector2D& pt1, const Vector2D& pt2, double u) {
		return Vector2D(pt1.x * (1.0 - u) + pt2.x * u, pt1.y * (1.0 - u) + pt2.y * u);
	}
};


//=============================================================================
// Extra Operations
//
// These are needed because in C++ the first argument of an operator is the one
// that the compiler looks at. For example, we can scale a point by multiplying
// by a number
//
//	Vector2D(2.0, 3.0) * 10.0;
//	10.0 * Vector2D(2.0, 3.0);
//
// The first one will call the member function, but to make the second one work we
// need the global function below. 

template <class T>
inline Vector2D<T> operator*(double scale, const Vector2D<T>& pt) {
	return Vector2D<T>(pt.x * scale, pt.y * scale);
}
