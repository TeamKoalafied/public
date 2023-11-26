//==============================================================================
// Bezier3_2D.h
//==============================================================================
#pragma once

#include "Vector2D.h"

// A 2d cubic Bezier curve segment
template <class T>
class Bezier3_2D
{
public:
	//==========================================================================
	// Properties

	Vector2D<T> m_point1;		// First point on the control polygon
	Vector2D<T> m_point2;		// Second point on the control polygon
	Vector2D<T> m_point3;		// Third point on the control polygon
	Vector2D<T> m_point4;		// Forth point on the control polygon


	//==========================================================================
	// Construction

	// Default constructor
	Bezier3_2D();

	// Point values constructor
	Bezier3_2D(const Vector2D<T>& point1, const Vector2D<T>& point2, const Vector2D<T>& point3, const Vector2D<T>& point4) :
        m_point1(point1),
        m_point2(point2),
        m_point3(point3),
        m_point4(point4) {
    }


	//==========================================================================
	// Operations

	// Evaluate the curve at the given parameter value
	//
	// u -  Parameter value to evaluate at. Range is [0.0, 1.0]
	//
	// Returns the point on the curve
	Vector2D<T> Evaluate(double u) const;

	// Evaluate the derivate of the curve at the given parameter value
	//
	// u - Parameter value to evaluate at. Range is [0.0, 1.0]</param>
	//
	// Returns the derivative of the curve
	Vector2D<T> Derivative(double u) const;

	// Evaluate the second derivate of the curve at the given parameter value
	//
	// u - Parameter value to evaluate at. Range is [0.0, 1.0]</param>
	//
	// Returns the second derivative of the curve
	Vector2D<T> Derivative2(double u) const;

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
	void Split(double u, Bezier3_2D<T>& bezier3_1, Bezier3_2D<T>& bezier3_2) const;

	// Calculate the length of the curve using subdivision
	//
	// error_ratio - Error ratio to stop the subdivision at (smaller is more precise, but slower)
	// max_depth - Maximum depth of subdivision (larger may be more precise and slower)
	//
	// Returns the length of the curve
	T Length(double error_ratio = 0.01, int max_depth = 5) const;

	// Calculate the position of a point a given arc distance along the curve
	//
	// arc_distance - Distance along the curve to calculate the point at
	// u - Returns the curve parameter value at the calculated point. This may be a poor approximatation but will
	// provide a position where a good approximation of the derivative can be obtained.
	// error_ratio - Error ratio to stop the subdivision at when calculating curve distance (smaller is more precise, but slower)
	// max_depth - Maximum depth of subdivision when calculating curve distance (larger may be more precise and slower)
	//
	// Returns the point on the curve
	Vector2D<T> ArcLengthPoint(T arc_distance, double& u, double error_ratio = 0.01, int max_depth = 5) const;
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
	bool ArcLengthSearch(T arc_distance, Vector2D<T>& result, double& u, T& total_length, double error_ratio, int max_depth) const;
};

//==========================================================================
// Construction

template <class T>
Bezier3_2D<T>::Bezier3_2D()
{
}


//==========================================================================
// Operations

template <class T>
Vector2D<T> Bezier3_2D<T>::Evaluate(double u) const
{
	Vector2D<T> q1 = Vector2D<T>::Lerp(m_point1, m_point2, u);
	Vector2D<T> q2 = Vector2D<T>::Lerp(m_point2, m_point3, u);
	Vector2D<T> q3 = Vector2D<T>::Lerp(m_point3, m_point4, u);

	Vector2D<T> r1 = Vector2D<T>::Lerp(q1, q2, u);
	Vector2D<T> r2 = Vector2D<T>::Lerp(q2, q3, u);

	return Vector2D<T>::Lerp(r1, r2, u);
}

template <class T>
Vector2D<T> Bezier3_2D<T>::Derivative(double u) const
{
	Vector2D<T> q1 = Vector2D<T>::Lerp(m_point1, m_point2, u);
	Vector2D<T> q2 = Vector2D<T>::Lerp(m_point2, m_point3, u);
	Vector2D<T> q3 = Vector2D<T>::Lerp(m_point3, m_point4, u);

	Vector2D<T> r1 = Vector2D<T>::Lerp(q1, q2, u);
	Vector2D<T> r2 = Vector2D<T>::Lerp(q2, q3, u);

	return 3.0*(r2 - r1);
}

template <class T>
Vector2D<T> Bezier3_2D<T>::Derivative2(double u) const
{
	Vector2D<T> q1 = Vector2D<T>::Lerp(m_point1, m_point2, u);
	Vector2D<T> q2 = Vector2D<T>::Lerp(m_point2, m_point3, u);
	Vector2D<T> q3 = Vector2D<T>::Lerp(m_point3, m_point4, u);

	return 6.0*(q3 - 2.0*q2 + q1);
}

template <class T>
double Bezier3_2D<T>::Curvature(double u) const
{
    // https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
    // https://en.wikiversity.org/wiki/CAGD/B%C3%A9zier_Curves
	Vector2D<T> d1 = Derivative(u);
	Vector2D<T> d2 = Derivative2(u);

    // NOTE: That this gives +ve curvature to the left
    // TODO Use proper units for curvature (1/T)
	return (double)((d1.x * d2.y - d1.y * d2.x)/units::math::sqrt(units::math::pow<3>(d1.x*d1.x + d1.y*d1.y)));
}

template <class T>
void Bezier3_2D<T>::Split(double u, Bezier3_2D& Bezier3_2D_1, Bezier3_2D& Bezier3_2D_2) const
{
	Bezier3_2D_1.m_point1 = m_point1;
	Bezier3_2D_2.m_point4 = m_point4;

	Bezier3_2D_1.m_point2 = Vector2D<T>::Lerp(m_point1, m_point2, u);
	Vector2D<T> q2 = Vector2D<T>::Lerp(m_point2, m_point3, u);
	Bezier3_2D_2.m_point3 = Vector2D<T>::Lerp(m_point3, m_point4, u);

	Bezier3_2D_1.m_point3 = Vector2D<T>::Lerp(Bezier3_2D_1.m_point2, q2, u);
	Bezier3_2D_2.m_point2 = Vector2D<T>::Lerp(q2, Bezier3_2D_2.m_point3, u);

	Bezier3_2D_1.m_point4 = Bezier3_2D_2.m_point1 = Vector2D<T>::Lerp(Bezier3_2D_1.m_point3, Bezier3_2D_2.m_point2, u);
}

template <class T>
T Bezier3_2D<T>::Length(double error_ratio, int max_depth) const
{
	// Calculate the length as the length of the control polygon
	T length = (m_point1 - m_point2).Length();
	length += (m_point2 - m_point3).Length();
	length += (m_point3 - m_point4).Length();

	// Calculate the chord distance between the start and end points
	T chord = (m_point1 - m_point4).Length();

	// If the length is within the error distance of the chord then return the length
	// as the length of the curve
	T error = length * error_ratio;
	if ((length - chord) <= error || max_depth < 0) return length;

	// Subdivide the curve and calculate the length as the length of the two pieces
	Bezier3_2D left;
	Bezier3_2D right;
	Split(0.5, left, right);
	return left.Length(error_ratio, max_depth - 1) + right.Length(error_ratio, max_depth - 1);
}

template <class T>
Vector2D<T> Bezier3_2D<T>::ArcLengthPoint(T arc_distance, double& u, double error_ratio, int max_depth) const
{
	T total_length;
	Vector2D<T> result;
	if (ArcLengthSearch(arc_distance, result, u, total_length, error_ratio, max_depth)) {
		return result;
	}
	if (units::math::abs(arc_distance - total_length) < error_ratio * total_length) {
		u = 1.0;
		return Evaluate(u);
	}

	//Debug.Assert(false);
	u = 0.0;
	return Vector2D<T>(T(0), T(0));
}

template <class T>
bool Bezier3_2D<T>::ArcLengthSearch(T arc_distance, Vector2D<T>& result, double& u, T& total_length, double error_ratio, int max_depth) const
{
	// Calculate the length as the length of the control polygon
	T length = (m_point1 - m_point2).Length();
	length += (m_point2 - m_point3).Length();
	length += (m_point3 - m_point4).Length();

	// Calculate the chord distance between the start and end points
	T chord = (m_point1 - m_point4).Length();

	// If the length is within the error distance of the chord then we regard this curve as a straight
	// line and can use linear interpolation of the curve parameter with distance.
	T error = length * error_ratio;
	if ((length - chord) <= error || max_depth < 0) {
		if (arc_distance > length) {
			// If the arc distance is beyond the length of the curve then return that we failed
			// to find the point.
			u = 0.0;
			result.Set(T(0), T(0));
			total_length = length;
			return false;
		}
		else {
			// Calculate the curve parameter using linear interpolation with distance.
			// Then calculate the actual point using linear interpolation between the
			// end points. This is critical as even if the Bezier points are colinear
			// the curve is not arc length parameterised.
			// Note that the value of 'u' returned can be quite inaccurate, but it fine for
			// calculating the derivative as this segment is approximately a straight line
			// and so all points have the same derivative.
			u = arc_distance / length;
            result = Vector2D<T>::Lerp(m_point1, m_point4, u);
			total_length = T(0);
			return true;
		}
	}

	// Subdivide the curve in the middle
	Bezier3_2D left;
	Bezier3_2D right;
	Split(0.5, left, right);

	// Search for the required point in the left half of the curve
	double left_u;
	T left_length;
	if (left.ArcLengthSearch(arc_distance, result, left_u, left_length, error_ratio, max_depth - 1)) {
		u = left_u / 2.0;
		total_length = T(0);
		return true;
	}

	// Search for the required point in the right half of the curve
	double right_u;
	T right_length;
	if (right.ArcLengthSearch(arc_distance - left_length, result, right_u, right_length, error_ratio, max_depth - 1)) {
		u = 0.5 + right_u / 2.0;
		total_length = T(0);
		return true;
	}

	// Fail to find the point anywhere
	u = 0.0;
	total_length = left_length + right_length;
	return false;
}
