//
// DWGeometry.h
//

#pragma once

class Point2D;

namespace Geometry
{

//==============================================================================
// Solving Linear Systems

// Solve the 2x2 linear system and return the matrix rank
int Solve2D(double a[2][3]);

// Solve an arbitrary linear system and return the matrix rank
// Line Intersection Tests

// Find the point of intersection of the two lines expressed in parameteric coordinate of the lines.
// Return the dimension of the intersection (-1, 0 or 1)
int LineIntersect(const Point2D& segment1_pt1, const Point2D& segment1_pt2,
				  const Point2D& segment2_pt1, const Point2D& segment2_pt2, 
				  double& line1_pos, double& line2_pos);


// Return the signed distance beween the point and the line defined
// by two points. Positive is to the right when 1 to 2 is up.
double DistancePointFromLine(const Point2D& pt, const Point2D& line_pt1, const Point2D& line_pt2);


// Project the point onto the line. Relative is the position along the line.
Point2D ProjectPointOnToLine(const Point2D& pt, const Point2D& line_pt1, const Point2D& line_pt2, double& relative);

// Return true if the points form a clockwise triangle
bool IsClockwiseTriangle(const Point2D& pt1, const Point2D& pt2, const Point2D& pt3);

} // namespace Geometry


