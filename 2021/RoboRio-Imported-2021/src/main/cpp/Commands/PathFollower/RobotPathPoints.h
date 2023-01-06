//==============================================================================
// RobotPathPoints.h
//==============================================================================
#pragma once

#include "../RobotPath/Point2D.h"
#include "../RobotPath/Bezier3.h"
#include "../RobotPath/MotionProfile.h"
#include <vector>

// RobotPathPoints takes a list of Bezier curve segments and produces two lists of
// points that track the required movement of the left and right side of the robot
// to follow the path.
// The calculation is quite fast so that paths can be generated on the fly.
class RobotPathPoints
{
public:
	//==========================================================================
	// Public Nested Types

	// PathPoint holds the details for one side of the robot at a single time point
	struct PathPoint {
		double m_time;
		Point2D m_position;
		double m_distance;
		double m_velocity;
		double m_acceleration;
		double m_jerk;
		double m_heading_deg;
	};


	//==========================================================================
	// Construction and Destruction

	// Constructor
	RobotPathPoints();

	// Destructor
	~RobotPathPoints();


	//==========================================================================
	// Path Generation

	// Generate the path
	//
	// path_definition - Path to follow as defined by a list of Bezier curve segments
	// motion_profile - Motion profile that defines how to move along the path
	// wheel_base - Width of the robot wheelbase
	// reverse - Whether the robot reverses along the path, or drives forward
	// time_period_s - Interval between the periodic updates when following the path
	void GeneratePathPoints(const std::vector<Bezier3>& path_definition, const MotionProfile& motion_profile,
							double wheel_base, bool reverse, double time_period_s);

	// Calculate the distance, velocity, acceleration and jerk from the the vector positions of the path points
	//
	// path_points - Path points to calculate for
	void CalculatePathParameters(std::vector<PathPoint>& path_points);

	// Get the number of points in the path (this is the same for the left and right sides)
	int GetTotalPoints() const  { return m_left_path_points.size(); }

	// Get the points for the left side of the robot
	const std::vector<PathPoint>& GetLeftPathPoints() const { return m_left_path_points; }

	// Get the points for the right side of the robot
	const std::vector<PathPoint>& GetRightPathPoints() const { return m_right_path_points; }


	//==============================================================================
	// Statis Test Function

	// Generate a points for a test path and log the results
	static void TestPathGeneration();
private:

	//==========================================================================
	// Member Variables

	std::vector<PathPoint> m_left_path_points;	// Points for the left side of the robot
	std::vector<PathPoint> m_right_path_points;	// Points for the right side of the robot
};

