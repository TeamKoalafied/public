//==============================================================================
// TestPaths.cpp
//==============================================================================

#include "TestPaths.h"

#include "ChallengePaths.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

#include "../RobotConfiguration.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

namespace RC = RobotConfiguration;

//==========================================================================
// Static Joystick Testing Control Functions

RobotPath* TestPaths::CreateTestPath(int pov_angle, double max_velocity, double max_acceleration)
{
	switch (pov_angle) {
		case RC::kJoystickPovUp: {
			std::cout << "Starting TestPaths - Straight\n";
			return CreateStraightPath(max_velocity, max_acceleration, 1.0);
		}
		case RC::kJoystickPovLeft: {
			std::cout << "Starting TestPaths - CreateTestCirclePath\n";
			return CreateTestCirclePath(max_velocity, max_acceleration);
		}
		// case RC::kJoystickPovDown: {
		// 	std::cout << "Starting TestPaths - Cube2\n";
		// 	//return CreateCube2Path(max_velocity, max_acceleration);
		// }
		case RC::kJoystickPovRight: {
			std::cout << "Starting TestPaths - From Dashboard\n";
			return CreateVisionPathFromDashBoard(max_velocity, max_acceleration);
		}
	}
    return NULL;
}


//==========================================================================
// Static Joystick Testing Implementation Functions

RobotPath* TestPaths::CreateVisionPathFromDashBoard(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
	double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
	// double targetArea = table->GetNumber("ta",0.0);
	//double targetSkew = table->GetNumber("ts",0.0);

	// Get the parameters from the SmartDashboard
    double vision_x = 0;
    double vision_y = 0;
    double vision_heading_degrees = 0.0;

	double kCameraHeight = 0.63; //0.26 when lowered
	double kTargetHeight = 2.17;
	double kCameraAngle = 24.55;
	double kWantedDistance = 2;
	
	// calculate distance
	double distanceFromTarget = (kTargetHeight - kCameraHeight) / 
								(tan((kCameraAngle + targetOffsetAngle_Vertical)*(M_PI/180)));
	vision_x = (distanceFromTarget) - kWantedDistance;
	vision_y = (distanceFromTarget * (tan(targetOffsetAngle_Horizontal * (M_PI/180)))) * -1;
	std::cout << "Vertical offset: " << targetOffsetAngle_Vertical << std::endl;
	std::cout << "Horizontal offset: " << targetOffsetAngle_Horizontal <<std::endl << std::endl;
	std::cout << "Distance: " << distanceFromTarget << std::endl << std::endl;
	std::cout << "Vision X: " << vision_x << std::endl;
	std::cout << "VIsion Y: " << vision_y << std::endl;
	// vision_heading_degrees = targetOffsetAngle_Horizontal;
	// vision_x=0;
	// vision_y=0;
	// Limit the values to a sensible range (could change, but prevents nasty behaviour while testing)
	if (vision_x < 0.0) vision_x = 0.0;
	// if (vision_x > 5.0) vision_x = 5.0;
	// if (vision_y < -3.0) vision_y = -3.0;
	
	// if (vision_y > 3.0)  vision_y =  3.0;

	// Start the path at the origin. This is arbitrary, so the origin is just simplest.
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	// Set the end of the path and calculate the straight line distance
	path.m_point4.Set(vision_x, vision_y);

	double distance = path.m_point4.Length();

	// The initial heading is 0 degrees so the second Bezier point is directly on the x axis.
	// We chose the distance to the second Bezier point be half the distance. This is can be changed
	// but seems to produce reasonable results
	path.m_point2.Set(distance / 2.0, 0.0);
	// The third Bezier point is calculated using the desired final heading.
	path.m_point3 = path.m_point4 - (distance / 2.0)*Point2D::UnitVectorDegrees(vision_heading_degrees);
	// Setup a path segment using the calculated Bezier
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Vision";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	robot_path->m_path_segments.push_back(path_segment);
	return robot_path;
}

RobotPath* TestPaths::CreateStraightPath(double max_velocity, double max_acceleration, double distance) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Straight2m";

	// Straight forward
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	path.m_point2.Set(distance*0.25, 0.0);
	path.m_point3.Set(distance*0.75, 0.0);
	path.m_point4.Set(distance, 0.0);			
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Straight";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	path_segment->m_reverse = false;
	//path_segment->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* TestPaths::CreateTestCirclePath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();
	robot_path->m_name = "TestCircle";

	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "TestCircle";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);


    double radius = frc::SmartDashboard::GetNumber("TestPathsRadiusM", 1.0);
    frc::SmartDashboard::PutNumber("TestPathsRadiusM", radius); 

    // Forwards 2m
    // Circle right with give radius
    // Forwards 1m
	ChallengePaths::AddStraight(path_segment, 2.0, Point2D(0, 0), Point2D(1.0, 0.0));
	ChallengePaths::AddTurnRight(path_segment, radius);
	ChallengePaths::AddTurnRight(path_segment, radius);
	ChallengePaths::AddTurnRight(path_segment, radius);
	ChallengePaths::AddTurnRight(path_segment, radius);
	ChallengePaths::AddStraight(path_segment, 1.0);

	return robot_path;
}
