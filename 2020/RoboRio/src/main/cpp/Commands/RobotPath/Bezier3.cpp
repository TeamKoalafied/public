//==============================================================================
// Bezier3.cpp
//==============================================================================

#include "Bezier3.h"


//==========================================================================
// Construction

Bezier3::Bezier3()
{
}


//==========================================================================
// Operations

Point2D Bezier3::Evaluate(double u) const
{
	Point2D q1 = Point2D::Lerp(m_point1, m_point2, u);
	Point2D q2 = Point2D::Lerp(m_point2, m_point3, u);
	Point2D q3 = Point2D::Lerp(m_point3, m_point4, u);

	Point2D r1 = Point2D::Lerp(q1, q2, u);
	Point2D r2 = Point2D::Lerp(q2, q3, u);

	return Point2D::Lerp(r1, r2, u);
}

Point2D Bezier3::Derivative(double u) const
{
	Point2D q1 = Point2D::Lerp(m_point1, m_point2, u);
	Point2D q2 = Point2D::Lerp(m_point2, m_point3, u);
	Point2D q3 = Point2D::Lerp(m_point3, m_point4, u);

	Point2D r1 = Point2D::Lerp(q1, q2, u);
	Point2D r2 = Point2D::Lerp(q2, q3, u);

	return (r2 - r1);
}

void Bezier3::Split(double u, Bezier3& bezier3_1, Bezier3& bezier3_2) const
{
	// double u_m_1 = 1.0 - u;

	bezier3_1.m_point1 = m_point1;
	bezier3_2.m_point4 = m_point4;

	bezier3_1.m_point2 = Point2D::Lerp(m_point1, m_point2, u);
	Point2D q2 = Point2D::Lerp(m_point2, m_point3, u);
	bezier3_2.m_point3 = Point2D::Lerp(m_point3, m_point4, u);

	bezier3_1.m_point3 = Point2D::Lerp(bezier3_1.m_point2, q2, u);
	bezier3_2.m_point2 = Point2D::Lerp(q2, bezier3_2.m_point3, u);

	bezier3_1.m_point4 = bezier3_2.m_point1 = Point2D::Lerp(bezier3_1.m_point3, bezier3_2.m_point2, u);
}

double Bezier3::Length(double error_ratio, int max_depth) const
{
	// Calculate the length as the length of the control polygon
	double length = (m_point1 - m_point2).Length();
	length += (m_point2 - m_point3).Length();
	length += (m_point3 - m_point4).Length();

	// Calculate the chord distance between the start and end points
	double chord = (m_point1 - m_point4).Length();

	// If the length is within the error distance of the chord then return the length
	// as the length of the curve
	double error = length * error_ratio;
	if ((length - chord) <= error || max_depth < 0) return length;

	// Subdivide the curve and calculate the length as the length of the two pieces
	Bezier3 left;
	Bezier3 right;
	Split(0.5, left, right);
	return left.Length(error_ratio, max_depth - 1) + right.Length(error_ratio, max_depth - 1);
}

Point2D Bezier3::ArcLengthPoint(double arc_distance, double& u, double error_ratio, int max_depth) const
{
	double total_length;
	Point2D result;
	if (ArcLengthSearch(arc_distance, result, u, total_length, error_ratio, max_depth)) {
		return result;
	}
	if (fabs(arc_distance - total_length) < error_ratio * total_length) {
		u = 1.0;
		return Evaluate(u);
	}

	//Debug.Assert(false);
	u = 0.0;
	return Point2D(0, 0);
}

bool Bezier3::ArcLengthSearch(double arc_distance, Point2D& result, double& u, double& total_length, double error_ratio, int max_depth) const
{
	// Calculate the length as the length of the control polygon
	double length = (m_point1 - m_point2).Length();
	length += (m_point2 - m_point3).Length();
	length += (m_point3 - m_point4).Length();

	// Calculate the chord distance between the start and end points
	double chord = (m_point1 - m_point4).Length();

	// If the length is within the error distance of the chord then we regard this curve as a straight
	// line and can use linear interpolation of the curve parameter with distance.
	double error = length * error_ratio;
	if ((length - chord) <= error || max_depth < 0) {
		if (arc_distance > length) {
			// If the arc distance is beyond the length of the curve then return that we failed
			// to find the point.
			u = 0.0;
			result.Set(0.0, 0.0);
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
            result = Point2D::Lerp(m_point1, m_point4, u);
			total_length = 0.0;
			return true;
		}
	}

	// Subdivide the curve in the middle
	Bezier3 left;
	Bezier3 right;
	Split(0.5, left, right);

	// Search for the required point in the left half of the curve
	double left_u;
	double left_length;
	if (left.ArcLengthSearch(arc_distance, result, left_u, left_length, error_ratio, max_depth - 1)) {
		u = left_u / 2.0;
		total_length = 0.0;
		return true;
	}

	// Search for the required point in the right half of the curve
	double right_u;
	double right_length;
	if (right.ArcLengthSearch(arc_distance - left_length, result, right_u, right_length, error_ratio, max_depth - 1)) {
		u = 0.5 + right_u / 2.0;
		total_length = 0.0;
		return true;
	}

	// Fail to find the point anywhere
	u = 0.0;
	total_length = left_length + right_length;
	return false;
}
