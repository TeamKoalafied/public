//
// Geometry.cpp
//

#include "Geometry.h"

#include "Point2D.h"

//=============================================================================
// Solving Linear Systems

// Solve the 2x2 linear system and return the matrix rank
int Geometry::Solve2D(double a[2][3]) {
	// TODO: Implement good linear equation solver

	// Pivot
	if (fabs(a[1][0]) > fabs(a[0][0])) {
		double temp;
		temp    = a[0][0];
		a[0][0] = a[1][0];
		a[1][0] = temp;
		temp    = a[0][1];
		a[0][1] = a[1][1];
		a[1][1] = temp;
		temp    = a[0][2];
		a[0][2] = a[1][2];
		a[1][2] = temp;
	}

	// Normalise
	a[0][1] /= a[0][0];
	a[0][2] /= a[0][0];

//	TRACE("Normalised System:\n");
//	TRACE("%8.3f  %8.3f  %8.3f\n", a[0][0], a[0][1], a[0][2]);
//	TRACE("%8.3f  %8.3f  %8.3f\n", a[1][0], a[1][1], a[1][2]);

	// Zero
	a[1][1] -= a[1][0]*a[0][1];
	a[1][2] -= a[1][0]*a[0][2];

	if (a[1][1] == 0.0) {
		return 1;
	}

	// Normalise
	a[1][2] /= a[1][1];

	// Backsubstitute
	a[0][2] -= a[0][1]*a[1][2];

	return 2;
}


//=============================================================================
// Intersection Tests

// Find the point of intersection of the two lines expressed in parameteric coordinate of the line. Return the dimension of the intersection (-1, 0 or 1)
int Geometry::LineIntersect(const Point2D& segment1_pt1, const Point2D& segment1_pt2,
                            const Point2D& segment2_pt1, const Point2D& segment2_pt2,
                            double& line1_pos, double& line2_pos) {
	double a[2][3];
    a[0][0] = segment1_pt2.x - segment1_pt1.x;
    a[1][0] = segment1_pt2.y - segment1_pt1.y;
    a[0][1] = segment2_pt1.x - segment2_pt2.x;
    a[1][1] = segment2_pt1.y - segment2_pt2.y;
    a[0][2] = segment2_pt1.x - segment1_pt1.x;
    a[1][2] = segment2_pt1.y - segment1_pt1.y;

	int rank = Solve2D(a);

	line1_pos = a[0][2];
	line2_pos = a[1][2];

	switch (rank) {
		default:
		case 0:
			// Totally degernerate intersection
			return -1;
		case 1:
			// Lines are parallel, either no intersection, or line lie on top of each other
			if (line2_pos == 0.0) return 1;
			else                  return -1;
		case 2:
			// Non parallel intersecting lines. Dimension of intersection is 0, a point
			return 0;
	}
}


//=============================================================================

double Geometry::DistancePointFromLine(const Point2D& pt, const Point2D& line_pt1, const Point2D& line_pt2) {
    // Return the signed distance beween the point and the line defined
    // by two points. Positive is to the right when 1 to 2 is up.
	double dx = line_pt2.x - line_pt1.x;
	double dy = line_pt2.y - line_pt1.y;

	double a[2][3];
	a[0][0] = dx;
	a[1][0] = dy;
	a[0][1] = dy;
	a[1][1] = -dx;
	a[0][2] = pt.x - line_pt1.x;
	a[1][2] = pt.y - line_pt1.y;
	Solve2D(a);

	return a[1][2] * sqrt(dx*dx + dy*dy);
}

Point2D Geometry::ProjectPointOnToLine(const Point2D& pt, const Point2D& line_pt1, const Point2D& line_pt2,
                                       double& relative) {
    // Project the point onto the line. Relative is the position along the line.
	double dx = line_pt2.x - line_pt1.x;
	double dy = line_pt2.y - line_pt1.y;

	double a[2][3];
	a[0][0] = dx;
	a[1][0] = dy;
	a[0][1] = dy;
	a[1][1] = -dx;
	a[0][2] = pt.x - line_pt1.x;
	a[1][2] = pt.y - line_pt1.y;
	Solve2D(a);

	relative = a[0][2];

	return Point2D::Lerp(line_pt1, line_pt2, relative);
}

// Return true if the points form a clockwise triangle
bool Geometry::IsClockwiseTriangle(const Point2D& pt1, const Point2D& pt2, const Point2D& pt3) {
	Point2D edge1_direction = pt2 - pt1;
	Point2D edge2_direction = pt3 - pt2;
	double cross_z = edge1_direction.x * edge2_direction.y - edge1_direction.y * edge2_direction.x;
	return cross_z > 0.0;
}

