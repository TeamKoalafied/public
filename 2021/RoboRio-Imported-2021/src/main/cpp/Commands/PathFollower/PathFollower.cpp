//==============================================================================
// PathFollower.cpp
//==============================================================================

#include "PathFollower.h"

#include "../RobotPath/Bezier3.h"
#include "../RobotPath/IPathDriveBase.h"
#include "../RobotPath/Point2D.h"
#include "../RobotPath/PathSegment.h"
#include "../RobotPath/RobotPath.h"

#include <iostream>



//==========================================================================
// Construction

PathFollower::PathFollower(RobotPath* robot_path, IPathDriveBase* drive_base,
						   IMechanismController* mechanism_controller, bool record_samples) {
	m_robot_path = robot_path;
	m_drive_base = drive_base;
	m_mechanism_controller = mechanism_controller;
	m_record_samples = record_samples;
}

PathFollower::~PathFollower() {
	delete m_robot_path;
}


//==========================================================================
// Trajectory Following

void PathFollower::Start() {
	// Set the brake mode to true so we stop quickly if required
	m_drive_base->SetBrakeMode(true);

	// Start the path follower for the whole path

	// Setup the first path segment
	m_path_segment_index = 0;
	m_path_finished = false;
	SetupPathSegment();

	// Start the overall robot path and then start the first segment
	StartPath();
	StartSegment();
}

void PathFollower::Follow() {
	// Continue following the current segment
	FollowSegment();

	// If the path segment has finished move to the next one
	if (IsSegmentFinished()) {
		// Log how far from the desired position we ended up
		// NMBTODO Remove this logging
  		// double position_x_m;
		// double position_y_m;
		// double heading_degrees;
	    // m_drive_base->GetPositionM(position_x_m, position_y_m, heading_degrees);
		// Point2D& last_point = m_robot_path->m_path_segments.back()->m_path_definition.back().m_point4;
		// std::cout << "Segment " << m_path_segment_index <<
		// 			 ": Final Position (" << position_x_m << ", " << position_y_m << ") " <<
		// 			 "Target Position (" << last_point.x << ", " << last_point.y << ")\n";

		// Tell the path follower the segment is finished
		FinishSegment();

		// Advance to the next path segment
		m_path_segment_index++;
		if (m_path_segment_index < (int)m_robot_path->m_path_segments.size()) {
			// If there is another segment then set it up and start following it
			SetupPathSegment();
			StartSegment();
		}
		else {
			// If there are not more segments we are done. Notify the path follower that
			// the whole path is done and then clean it up.
			FinishPath();
			m_path_finished = true;
		}
	}
}

bool PathFollower::IsTrajectoryFinished() {
    // The command is finished if the list of path segments is finished
    return m_path_segment_index >= (int)m_robot_path->m_path_segments.size();
}

void PathFollower::Finish() {
	if (!m_path_finished) {
		FinishPath();
		m_path_finished = true;
	}
}

//==========================================================================
// Properties

PathSegment& PathFollower::GetPathSegment() {
	return *(m_robot_path->m_path_segments[m_path_segment_index]);
}


//==========================================================================
// Path Segment Setup

void PathFollower::SetupPathSegment() {
	if (m_path_segment_index == 0) {
		// For the first path segment we reset the dead reckoning position in the drivebase so
		// that it tracks the position along the path.

        // It is possible to have path segments with no path definition (robot is stationary
        // for the segment) so skip over those for the first segment that hase a path.
        int segment_index = m_path_segment_index;
        while (segment_index < (int)m_robot_path->m_path_segments.size() && 
               m_robot_path->m_path_segments[segment_index]->m_path_definition.empty()) {
            segment_index++;
        }
        if (segment_index >= (int)m_robot_path->m_path_segments.size()) return;

        // Get the start direction of the segment from the first two control points
	    PathSegment& path_segment = *(m_robot_path->m_path_segments[segment_index]);
		const Point2D start_point  = path_segment.m_path_definition[0].m_point1;
		const Point2D second_point = path_segment.m_path_definition[0].m_point2;
		double heading_degrees = (second_point - start_point).AngleDegrees();
		if (path_segment.m_reverse) {
			heading_degrees += 180; 
			while (heading_degrees >=  180.0) heading_degrees -= 360.0;
			while (heading_degrees <  -180.0) heading_degrees += 360.0;
		}

        // Reset the drivebase dead reckoning position (in inches)
		m_drive_base->ResetPosition(start_point.x/0.0254, start_point.y/0.0254, heading_degrees);
	} else {
		// For later path segments we adjust the beginning of the segment to take into account
		// where we actually finished the previous segment.
        PathSegment& path_segment = *(m_robot_path->m_path_segments[m_path_segment_index]);
		AdjustPathSegmentStart(path_segment);
	}
}

void PathFollower::AdjustPathSegmentStart(PathSegment& path_segment) {
    // Nothing to adjust if there is no path definition (robot is stationary for the segment)
    if (path_segment.m_path_definition.empty()) return;

	// Get the current position from the drive base
	double position_x_m;
	double position_y_m;
	double heading_degrees;
	m_drive_base->GetPositionM(position_x_m, position_y_m, heading_degrees);

	Bezier3& bezier = path_segment.m_path_definition[0];
	double point2_distance = (bezier.m_point2 - bezier.m_point1).Length();

	Point2D point1(position_x_m, position_y_m);
	Point2D point2;
	if (path_segment.m_reverse) {
		point2 = point1 - point2_distance*Point2D::UnitVectorDegrees(heading_degrees);
	} else {
		point2 = point1 + point2_distance*Point2D::UnitVectorDegrees(heading_degrees);
	}
	// NMBTODO Remove this logging
	std::cout << "PathFollower::AdjustPathSegmentStart\n";
	std::cout << "Point1 (" << bezier.m_point1.x << ", " << bezier.m_point1.y <<  ") to (" << point1.x << ", " << point1.y <<  ")\n";
	std::cout << "Point2 (" << bezier.m_point2.x << ", " << bezier.m_point2.y <<  ") to (" << point2.x << ", " << point2.y <<  ")\n";
	bezier.m_point1 = point1;
	bezier.m_point2 = point2;
}

