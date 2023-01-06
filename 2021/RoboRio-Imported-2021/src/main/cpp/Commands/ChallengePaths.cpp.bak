#include "ChallengePaths.h"
#include "../RobotConfiguration.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

#include "PathFollower/PathfinderFollower.h"
#include "PathFollower/PathPointsFollower.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <sstream>

namespace RC = RobotConfiguration;


//==========================================================================
// Static Joystick Testing Control Functions

RobotPath* ChallengePaths::CreateTestPath(int pov_angle, double max_velocity, double max_acceleration)
{
	// If the POV is pressed start one of the test paths
	RobotPath* robot_path = NULL;
	switch (pov_angle) {
		case RC::kJoystickPovUp: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::Straight\n";
			robot_path = CreateStraightPath(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovLeft: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::Slalom\n";
			robot_path = CreateSlalomPath(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovDown: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::CreateBarrelRacingPathRightAngles\n";
			robot_path = CreateBarrelRacingPathRightAngles(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovRight: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::CreateGalaticSearchPath\n";
			robot_path = CreateGalaticSearchPath(max_velocity, max_acceleration);
			break;
		}

	}

	// Append the max velocity and acceleration to the name so it gets logged
//	std::ostringstream full_name;
//  	full_name << robot_path->m_name << " MaxV: " << max_velocity << " MaxA: " << max_acceleration;
//  	robot_path->m_name = full_name.str();

    return robot_path;
}


//==============================================================================
// Path Creation

RobotPath* ChallengePaths::CreateStraightPath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Straight2m";

	// Straight 2m forward
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	path.m_point2.Set(0.5, 0.0);
	path.m_point3.Set(1.5, 0.0);
	path.m_point4.Set(2.0, 0.0);			
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Straight";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	path_segment->m_reverse = false;
//	path_segment->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* ChallengePaths::CreateSlalomPath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "SlalomRA";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Slalom";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);


	const double length = 60 * 0.0254;
	const double width = 70 * 0.0254;
	const double robot_length = 37 * 0.0254;

	// Create a path where the robot does the slalom in straight sections and right angle turns.
	// This might not work very well but is a useful experiment.
 	

	// Alternative method for creating the path
	AddStraight(path_segment, robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddSlalomLeft(path_segment, length, width, 0.5);
	AddStraight(path_segment, 2 * length);
	AddSlalomRight(path_segment, length, width, 0.5);
	AddTurnLeft(path_segment, width /2);
	AddTurnLeft(path_segment, width /2);
	AddSlalomLeft(path_segment, length, width, 0.5);
	AddStraight(path_segment, 2 * length);
	AddSlalomRight(path_segment, length, width, 0.5);
	AddStraight(path_segment, robot_length);
	
	return robot_path;
}

RobotPath* ChallengePaths::CreateSlalomPathRightAngles(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "SlalomRA";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Slalom";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);


	const double length = 60 * 0.0254;
	const double width = 70 * 0.0254;
	const double radius = 25 * 0.0254;
	const double robot_length = 37 * 0.0254;

	// Create a path where the robot does the slalom in straight sections and right angle turns.
	// This might not work very well but is a useful experiment.
 	

	// Alternative method for creating the path
	double start_length = length /2 - radius;
	AddStraight(path_segment, start_length + robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, width - 2*radius);
	AddTurnRight(path_segment, radius);
	AddStraight(path_segment, 2*length + 2*start_length);
	AddTurnRight(path_segment, radius);
	
	AddStraight(path_segment, width - 2*radius);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, 2*start_length);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, width - 2*radius); // Far end
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, 2*start_length);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, width - 2*radius);
	AddTurnRight(path_segment, radius);
	AddStraight(path_segment, 2*length + 2*start_length);
	AddTurnRight(path_segment, radius);
	AddStraight(path_segment, width - 2*radius);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, start_length + robot_length);


	return robot_path;
}

RobotPath* ChallengePaths::CreateBarrelRacingPathRightAngles(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "BarrelRacingRA";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "BarrelRacingRA";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);


	const double INCH = 0.0254;
	const double radius = 30 * INCH;
	const double robot_length = 37 * INCH;

    // Barrel Racing Layout
    //
    //                /---\                        .
    // 300"          |  *  |  3rd barrel, turn right
    //          /---\|     |
    // 240"    |  *  |     |  2nd barrel, turn left
    //          \----|----/
    //               |/---\                        .
    // 150"          |  *  |  1st barrel, trun right
    //               |\---/
    //               |
    // 60"        +  |  +
    //            +     +
    // 0"         +     +

	// Forwards and right around the first barrel
	AddStraight(path_segment, (150 - 60)*INCH + robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddTurnRight(path_segment, radius);
	AddTurnRight(path_segment, radius);
	AddTurnRight(path_segment, radius);
	AddTurnRight(path_segment, radius);

    // Forwards and left 3/4 around the second barrel
	AddStraight(path_segment, (240 - 150)*INCH);
	AddTurnLeft(path_segment, radius);
	AddTurnLeft(path_segment, radius);
	AddTurnLeft(path_segment, radius);

    // Forwards, single right and up to the third barrel
	AddStraight(path_segment, 60*INCH);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, (300 - 240)*INCH);
	
    // Left 1/2 turn around the third barrel and back to the finish zone
	AddTurnLeft(path_segment, radius);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, (300 - 60)*INCH + robot_length);

	return robot_path;
}

RobotPath* ChallengePaths::CreateBouncePath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Bounce";


	const double INCH = 0.0254;
	const double radius = 30 * INCH;
	const double robot_length = 37 * INCH;


	// Forwards and turn to first bump
	PathSegment* path_segment1 = new PathSegment();
	path_segment1->m_name = "Bounce";
	path_segment1->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment1->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment1);
	AddStraight(path_segment1, (150 - 60)*INCH + robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddTurnLeft(path_segment1, radius);
	AddStraight(path_segment1, 40*INCH);

	// Reverse to line up for second bump
	PathSegment* path_segment2 = new PathSegment();
	path_segment2->m_name = "Bounce";
	path_segment2->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment2->m_reverse = true;
	robot_path->m_path_segments.push_back(path_segment2);
    Point2D start;
    Point2D direction;
   	GetCurrent(path_segment1, start, direction);
    direction = -direction;
	AddStraight(path_segment1, 40*INCH, start, direction);
	AddSlalomLeft(path_segment1, 120*INCH, (150 - 90)*INCH, 0.5);

	// Forwards to second bump
	PathSegment* path_segment3 = new PathSegment();
	path_segment3->m_name = "Bounce";
	path_segment3->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment3->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment3);
   	GetCurrent(path_segment2, start, direction);
    direction = -direction;
	AddSlalomLeft(path_segment3, 120*INCH, (180 - 150)*INCH, 0.5, start, direction);
	AddStraight(path_segment3, 40*INCH);

	// Reverse to line up for third bump
	PathSegment* path_segment4 = new PathSegment();
	path_segment4->m_name = "Bounce";
	path_segment4->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment4->m_reverse = true;
	robot_path->m_path_segments.push_back(path_segment4);
   	GetCurrent(path_segment3, start, direction);
    direction = -direction;
	AddStraight(path_segment4, 40*INCH, start, direction);
	AddSlalomLeft(path_segment4, 120*INCH, (225 - 180)*INCH, 0.5);

	// Forwards to third bump
	PathSegment* path_segment5 = new PathSegment();
	path_segment5->m_name = "Bounce";
	path_segment5->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment5->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment5);
   	GetCurrent(path_segment4, start, direction);
    direction = -direction;
	AddSlalomLeft(path_segment5, 120*INCH, (270 - 225)*INCH, 0.5, start, direction);
	AddStraight(path_segment5, 40*INCH);

	// Reverse to finish
	PathSegment* path_segment6 = new PathSegment();
	path_segment6->m_name = "Bounce";
	path_segment6->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment6->m_reverse = true;
	robot_path->m_path_segments.push_back(path_segment6);
   	GetCurrent(path_segment5, start, direction);
    direction = -direction;
	AddStraight(path_segment6, 30*INCH, start, direction);
	AddTurnLeft(path_segment6, 40*INCH);
	AddStraight(path_segment6, robot_length);

	return robot_path;
}

RobotPath* ChallengePaths::CreateTestPath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "SlalomRA";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Slalom";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = true;
	robot_path->m_path_segments.push_back(path_segment);


	AddStraight(path_segment, 0.5, Point2D(0, 0), Point2D(1.0, 0.0));
	AddTurnLeft(path_segment, 1.0);

	return robot_path;
}


RobotPath* ChallengePaths::CreateGalaticSearchPath(double max_velocity, double max_acceleration)
{
	static MechanismAction START_INTAKE[] = {
			{ "StartIntaking",     MechanismAction::TimeSpecification::Start, 0.5, 0 }
	};

	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "GalaticSearch1";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "GalaticSearch1";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);

	path_segment->m_mechanism_actions.assign(START_INTAKE, START_INTAKE +  sizeof(START_INTAKE)/sizeof(START_INTAKE[0]));


	const double INCH = 0.0254;
	const double radius = 30 * INCH;
	const double robot_length = 37 * INCH;


	AddStraight(path_segment, (90 - 30)*INCH + robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddSlalomRight(path_segment, (150 - 90)*INCH, 30 * INCH, 0.5);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, 60 * INCH);
	AddTurnRight(path_segment, radius);
	AddStraight(path_segment, (360 - 180) * INCH - radius);

	return robot_path;
}

//==============================================================================
// Path Building

void ChallengePaths::AddStraight(PathSegment* path_segment, double length)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddStraight(path_segment, length, start, direction);
}

void ChallengePaths::AddTurnLeft(PathSegment* path_segment, double radius)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddTurnLeft(path_segment, radius, start, direction);
}

void ChallengePaths::AddTurnRight(PathSegment* path_segment, double radius)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddTurnRight(path_segment, radius, start, direction);
}

void ChallengePaths::AddSlalomLeft(PathSegment* path_segment, double length, double width, double fraction)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
    AddSlalomLeft(path_segment, length, width, fraction, start, direction);
}

void ChallengePaths::AddSlalomRight(PathSegment* path_segment, double length, double width, double fraction)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
    AddSlalomRight(path_segment, length, width, fraction, start, direction
	);
}

void ChallengePaths::GetCurrent(PathSegment* path_segment, Point2D& position, Point2D& direction)
{
	const Bezier3& path = path_segment->m_path_definition.back();
	position = path.m_point4;
	direction = path.m_point4 - path.m_point3; 
}

void ChallengePaths::AddStraight(PathSegment* path_segment, double length, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * (0.25 * length);
	path.m_point3 = start + unit_direction * (0.75 * length);
	path.m_point4 = start + unit_direction * length;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddTurnLeft(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * radius;
	path.m_point3 = start + unit_direction * radius;
	path.m_point4 = start + unit_direction * radius + TurnLeft(unit_direction) * radius;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddTurnRight(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * radius;
	path.m_point3 = start + unit_direction * radius;
	path.m_point4 = start + unit_direction * radius + TurnRight(unit_direction) * radius;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddSlalomLeft(PathSegment* path_segment, double length, double width, double fraction,
                                   const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

    Point2D fraction_vector = fraction * unit_direction;
    Point2D end = start + unit_direction * length + TurnLeft(unit_direction) * width;

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + fraction_vector;
	path.m_point3 = end - fraction_vector;
	path.m_point4 = end;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddSlalomRight(PathSegment* path_segment, double length, double width, double fraction,
                                    const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

    Point2D fraction_vector = fraction * unit_direction;
    Point2D end = start + unit_direction * length + TurnRight(unit_direction) * width;

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + fraction_vector;
	path.m_point3 = end - fraction_vector;
	path.m_point4 = end;

	path_segment->m_path_definition.push_back(path);
}

//==============================================================================
// Utility Helpers

Point2D ChallengePaths::TurnLeft(const Point2D& direction)
{
	return Point2D(-direction.y, direction.x);
}

Point2D ChallengePaths::TurnRight(const Point2D& direction)
{
	return Point2D(direction.y, -direction.x);
}
