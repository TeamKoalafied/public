//==============================================================================
// DrivePathFollower.cpp
//==============================================================================

#include "DrivePathFollower.h"

#include "MechanismController2020.h"
#include "../RobotConfiguration.h"
#include "../Subsystems/DriveBase.h"
#include "../Subsystems/Manipulator.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

#include "PathFollower/PathfinderFollower.h"
#include "PathFollower/PathPointsFollower.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

namespace RC = RobotConfiguration;


//==========================================================================
// Static Member Variables

DrivePathFollower* DrivePathFollower::ms_test_command = NULL;
VisionFindTarget* ms_find_target_command = NULL;



//==========================================================================
// Construction

DrivePathFollower::DrivePathFollower(PathFollower* path_follower_controller):
	frc::Command("DrivePathFollower") {

	// Driving requires the DriveBase and Manipulator
    Requires(&DriveBase::GetInstance());
    Requires(&Manipulator::GetInstance());

	m_path_follower = path_follower_controller;
}

// Destructor
DrivePathFollower::~DrivePathFollower() {
	delete m_path_follower;
}


//==========================================================================
// Function Overrides from frc::Command

void DrivePathFollower::Initialize() {
	// Called just before this Command runs the first time
	m_path_follower->Start();
}

void DrivePathFollower::Execute() {
	// Called repeatedly when this Command is scheduled to run
	m_path_follower->Follow();
}

bool DrivePathFollower::IsFinished() {
	// Make this return true when this Command no longer needs to run execute()
    if (IsTimedOut()) {
        printf("DrivePathFollower command timed out\n");
        return true;
    }

    // The command is finished if the trajectory is finished
    return m_path_follower->IsTrajectoryFinished();
}

void DrivePathFollower::End() {
	// Called once after isFinished returns true

	// Stop the drivebase so that the robot stops quickly. Especially important if cancelled.
	DriveBase& drive_base = DriveBase::GetInstance();
	drive_base.Stop();

	// Write the test data. We want this to happen even if the command is cancelled
	m_path_follower->Finish();
}

void DrivePathFollower::Interrupted() {
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	End();
}


//==========================================================================
// Static Joystick Testing Control Functions

void DrivePathFollower::DoJoystickTestControl(frc::Joystick* joystick)
{
	// Get the maximum velocity and acceleration from the dashboard
	double max_velocity = frc::SmartDashboard::GetNumber("AutoMaxV", 1.0);
	if (max_velocity < 0.1) max_velocity = 0.1;
	if (max_velocity > 3.0) max_velocity = 3.0;
	double max_acceleration = frc::SmartDashboard::GetNumber("AutoMaxA", 0.5);
	if (max_acceleration < 0.1) max_acceleration = 0.1;
	if (max_acceleration > 3.0) max_acceleration = 3.0;

	// If the POV is pressed start one of the test paths
	RobotPath* robot_path = NULL;
	int pov_angle = joystick->GetPOV(0);
	switch (pov_angle) {
		case RC::kJoystickPovUp: {
			std::cout << "Starting DrivePathFollower - Straight\n";
			robot_path = CreateStraightPath(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovLeft: {
			std::cout << "Starting DrivePathFollower - Ball1\n";
			robot_path = CreateBall1Path(max_velocity, max_acceleration);;
			break;
		}
		//case RC::kJoystickPovDown: {
		//	std::cout << "Starting DrivePathFollower - Ball2\n";
		//	robot_path = CreateBall2Path(max_velocity, max_acceleration);
		//	break;
		//}
		case RC::kJoystickPovDown: {
        	std::cout << "Rotating to target using VisionFindTarget command";                                                                        
            delete ms_find_target_command;
            ms_find_target_command = new VisionFindTarget();
            ms_find_target_command->Start();
            break;
        }

		
		case RC::kJoystickPovRight: {
			std::cout << "Starting DrivePathFollower - From Dashboard\n";
			robot_path = CreateVisionPathFromDashBoard(max_velocity, max_acceleration);
			break;
		}

	}

	// If there is a path create a follower and a command to do it
	if (robot_path != NULL) {
		PathFollower* path_follower = CreatePathPointsFollower(robot_path);
//		PathFollower* path_follower = CreatePathfinderFollower(robot_path);

		delete ms_test_command;
		ms_test_command = new DrivePathFollower(path_follower);
		ms_test_command->Start();
	}

	// If the 'B' button is pressed reset the drive base dead reckoning position
    if (joystick->GetRawButtonPressed(RobotConfiguration::kJoystickBButton)) {
		std::cout << "Reseting dead reckoning position\n";
		DriveBase& drive_base = DriveBase::GetInstance();
		drive_base.ResetPosition();
	}
}


//==========================================================================
// Static Joystick Testing Implementation Functions

PathFollower* DrivePathFollower::CreatePathPointsFollower(RobotPath* robot_path) {
	double p_gain = frc::SmartDashboard::GetNumber("AutoP", 1.0);
	if (p_gain < 0.5) p_gain = 0.5;
	if (p_gain > 3.0) p_gain = 3.0;
	double i_gain = frc::SmartDashboard::GetNumber("AutoI", 0.0);
	if (i_gain < 0.0) i_gain = 0.0;
	if (i_gain > 3.0) i_gain = 3.0;
	double d_gain = frc::SmartDashboard::GetNumber("AutoD", 0.0);
	if (d_gain < 0.0) d_gain = 0.0;
	if (d_gain > 3.0) d_gain = 3.0;


	DriveBase& drive_base = DriveBase::GetInstance();
	PathPointsFollower* path_follower = new PathPointsFollower(robot_path, &drive_base, new MechanismController2020, true);
	path_follower->GetFollowerParameters().m_kp = p_gain;
	path_follower->GetFollowerParameters().m_ki = i_gain;
	path_follower->GetFollowerParameters().m_kd = d_gain;
	path_follower->GetFollowerParameters().m_kv = 1.0/4.28;
	path_follower->GetFollowerParameters().m_kv_offset = 0.104; // 0.05719;
	path_follower->GetFollowerParameters().m_ka = 1.0/18.29;
	path_follower->GetFollowerParameters().m_wheelbase_width_m = 0.64;
	path_follower->GetFollowerParameters().m_period_s = 0.02;
	return path_follower;
}

PathFollower* DrivePathFollower::CreatePathfinderFollower(RobotPath* robot_path) {
	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V
	EncoderConfig encode_config;
	encode_config.initial_position = 0;
	encode_config.ticks_per_revolution = 4096;
	encode_config.wheel_circumference = M_PI * 6.25 * 0.0254;
	encode_config.kp = 1.0;
	encode_config.ki = 0.0;
	encode_config.kd = 0.0;
	encode_config.kv = 1.0/4.28;
	encode_config.ka = 1.0/18.29;

	DriveBase& drive_base = DriveBase::GetInstance();

	return new PathfinderFollower(robot_path, &drive_base, new MechanismController2020, true, encode_config);
}

RobotPath* DrivePathFollower::CreateVisionPathFromDashBoard(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	// Get the parameters from the SmartDashboard
    double vision_x = frc::SmartDashboard::GetNumber("VisionTrackX", 3.0);
    double vision_y = frc::SmartDashboard::GetNumber("VisionTrackY", 0.0);
    double vision_heading_degrees = frc::SmartDashboard::GetNumber("VisionTrackHeading", 0.0);

	// Limit the values to a sensible range (could change, but prevents nasty behaviour while testing)
	if (vision_x < 0.0) vision_x = 0.0;
	if (vision_x > 5.0) vision_x = 5.0;
	if (vision_y < -3.0) vision_y = -3.0;
	
	if (vision_y > 3.0)  vision_y =  3.0;

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

// static MechanismAction GRAB_CUBE[] = {
//		{ "DropArm",    MechanismAction::TimeSpecification::End, -2.5, 0 },
//		{ "OpenClaw",   MechanismAction::TimeSpecification::End, -2.0, 0 },
//		{ "RollerGrab", MechanismAction::TimeSpecification::End, -1.0, 0 },
//		{ "CloseClaw",  MechanismAction::TimeSpecification::End, -0.5, 0 },
//		{ "LiftArm",    MechanismAction::TimeSpecification::End,  0.0, 0 },
//};
//static MechanismAction PLACE_CUBE[] = {
//		{ "DropArm",     MechanismAction::TimeSpecification::End, -1.5, 0 },
//		{ "OpenClaw",    MechanismAction::TimeSpecification::End,  0.0, 0 },
//		{ "RollerPlace", MechanismAction::TimeSpecification::End,  0.0, 0 },
//};
//static MechanismAction RESET[] = {
//		{ "CloseClaw",   MechanismAction::TimeSpecification::Start, 1.0, 0 },
//		{ "LiftArm",     MechanismAction::TimeSpecification::Start, 1.0, 0 },
//};	

static MechanismAction INTAKE_BALL[] = {
		 { "ExtendIntake", MechanismAction::TimeSpecification::Start, 1.0, 0 },
		 { "RunIndexForward", MechanismAction::TimeSpecification::Start, 1.0, 0 },
		 { "RetractIntake", MechanismAction::TimeSpecification::Start, 2.0, 0 },
};

static MechanismAction SHOOT_BALL[] = {
		{ "Shoot",     MechanismAction::TimeSpecification::End, -1.5, 0 },
};

static MechanismAction RESET[] = {
		{ "RetractIntake", MechanismAction::TimeSpecification::Start, 2.0, 0 },
};


RobotPath* DrivePathFollower::CreateStraightPath(double max_velocity, double max_acceleration) {
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
	path_segment->m_mechanism_actions.assign(INTAKE_BALL, INTAKE_BALL +  sizeof(INTAKE_BALL)/sizeof(INTAKE_BALL[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* DrivePathFollower::CreateBall1Path(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();
	robot_path->m_name = "Ball1";

	const double TL = 1.5;
	Bezier3 path1;
	path1.m_point1.Set( 0.0,  0.0);
	path1.m_point2.Set( 0.5,  0.0);
	path1.m_point3.Set( 2.5,  0.0);
	path1.m_point4.Set( 3.0,  0.0);
	Bezier3 path2;
	path2.m_point1.Set( 3.0,  0.0);
	path2.m_point2.Set( 3.0-TL,  0.0);
	path2.m_point3.Set( 1.5,  -1.5+TL);
	path2.m_point4.Set( 1.5, -1.5);			
	Bezier3 path3;
	path3.m_point1.Set( 1.5, -1.5);
	path3.m_point2.Set( 1.5,  -1.5+TL);
	path3.m_point3.Set( TL,  0.0);
	path3.m_point4.Set( 0.0,  0.0);


	PathSegment* path_segment1 = new PathSegment();
	path_segment1->m_name = "GrabBall";
	path_segment1->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment1->m_path_definition.push_back(path1);
	path_segment1->m_mechanism_actions.assign(INTAKE_BALL, INTAKE_BALL +  sizeof(INTAKE_BALL)/sizeof(INTAKE_BALL[0]));
	robot_path->m_path_segments.push_back(path_segment1);

	PathSegment* path_segment2 = new PathSegment();
	path_segment2->m_name = "Reverse";
	path_segment2->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment2->m_reverse = true;
	path_segment2->m_path_definition.push_back(path2);
	robot_path->m_path_segments.push_back(path_segment2);

	PathSegment* path_segment3 = new PathSegment();
	path_segment3->m_name = "ShootBall";
	path_segment3->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment3->m_path_definition.push_back(path3);
	path_segment3->m_mechanism_actions.assign(SHOOT_BALL, SHOOT_BALL +  sizeof(SHOOT_BALL)/sizeof(SHOOT_BALL[0]));
	robot_path->m_path_segments.push_back(path_segment3);

	return robot_path;
}

RobotPath* DrivePathFollower::CreateBall2Path(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();
	robot_path->m_name = "Ball2";

	Bezier3 path1;
	path1.m_point1.x =  0.0;
	path1.m_point1.y =  0.0;
	path1.m_point2.x =  1.5;
	path1.m_point2.y =  0.0;
	path1.m_point3.x =  1.5;
	path1.m_point3.y =  0.0;
	path1.m_point4.x =  1.5;
	path1.m_point4.y = -1.5;
	Bezier3 path2;
	path2.m_point1.x =  1.5;
	path2.m_point1.y = -1.5;
	path2.m_point2.x =  1.5;
	path2.m_point2.y =  0.0;
	path2.m_point3.x =  1.5;
	path2.m_point3.y =  1.0;
	path2.m_point4.x =  3.0;
	path2.m_point4.y =  1.0;			
	Bezier3 path3;
	path3.m_point1.x =  3.0;
	path3.m_point1.y =  1.0;
	path3.m_point2.x =  1.5;
	path3.m_point2.y =  1.0;
	path3.m_point3.x =  1.5;
	path3.m_point3.y =  0.0;
	path3.m_point4.x =  1.5;
	path3.m_point4.y = -1.5;			
	Bezier3 path4;
	path4.m_point1.x =  1.5;
	path4.m_point1.y = -1.5;
	path4.m_point2.x =  1.5;
	path4.m_point2.y =  0.0;
	path4.m_point3.x =  1.5;
	path4.m_point3.y =  0.0;
	path4.m_point4.x =  0.0;
	path4.m_point4.y =  0.0;


	PathSegment* path_segment1 = new PathSegment();
	path_segment1->m_name = "Ball2_1";
	path_segment1->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment1->m_reverse = true;
	path_segment1->m_path_definition.push_back(path1);
	path_segment1->m_mechanism_actions.assign(RESET, RESET +  sizeof(RESET)/sizeof(RESET[0]));
	robot_path->m_path_segments.push_back(path_segment1);

	PathSegment* path_segment2 = new PathSegment();
	path_segment2->m_name = "Ball2_2";
	path_segment2->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment2->m_path_definition.push_back(path2);
	path_segment2->m_mechanism_actions.assign(INTAKE_BALL, INTAKE_BALL +  sizeof(INTAKE_BALL)/sizeof(INTAKE_BALL[0]));
	robot_path->m_path_segments.push_back(path_segment2);

	PathSegment* path_segment3 = new PathSegment();
	path_segment3->m_name = "Ball2_3";
	path_segment3->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment3->m_reverse = true;
	path_segment3->m_path_definition.push_back(path3);
	robot_path->m_path_segments.push_back(path_segment3);

	PathSegment* path_segment4 = new PathSegment();
	path_segment4->m_name = "Ball2_4";
	path_segment4->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment4->m_path_definition.push_back(path4);
	path_segment4->m_mechanism_actions.assign(SHOOT_BALL, SHOOT_BALL +  sizeof(SHOOT_BALL)/sizeof(SHOOT_BALL[0]));
	robot_path->m_path_segments.push_back(path_segment4);

	return robot_path;
}
