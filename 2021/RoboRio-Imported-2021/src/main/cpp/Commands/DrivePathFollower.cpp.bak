//==============================================================================
// DrivePathFollower.cpp
//==============================================================================

#include "DrivePathFollower.h"

#include "MechanismController2020.h"
#include "ChallengePaths.h"
#include "TestPaths.h"
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
#include "PathFollower/PurePursuitFollower.h"

#include "VisionFindTarget.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
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

	// Driving requires the DriveBase and elevator
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
	robot_path = ChallengePaths::CreateTestPath(pov_angle, max_velocity, max_acceleration);
	//robot_path = TestPaths::CreateTestPath(pov_angle, max_velocity, max_acceleration);

	// If there is a path create a follower and a command to do it
	if (robot_path != NULL) {
//		PathFollower* path_follower = CreatePathPointsFollower(robot_path);
//		PathFollower* path_follower = CreatePathfinderFollower(robot_path);
		PathFollower* path_follower = CreatePurePursuitFollower(robot_path, max_velocity, max_acceleration);

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

PathFollower* DrivePathFollower::CreatePurePursuitFollower(RobotPath* robot_path, double max_velocity, double max_acceleration) {
	double p_gain = frc::SmartDashboard::GetNumber("AutoP", 0.3);
	if (p_gain < 0.1) p_gain = 0.1;
	if (p_gain > 3.0) p_gain = 3.0;
	double i_gain = frc::SmartDashboard::GetNumber("AutoI", 0.0);
	if (i_gain < 0.0) i_gain = 0.0;
	if (i_gain > 3.0) i_gain = 3.0;
	double d_gain = frc::SmartDashboard::GetNumber("AutoD", 0.0);
	if (d_gain < 0.0) d_gain = 0.0;
	if (d_gain > 3.0) d_gain = 3.0;
	double curvature_gain = frc::SmartDashboard::GetNumber("AutoCurveGain", 1.5);
	if (curvature_gain < 1.0) curvature_gain = 1.0;
	if (curvature_gain > 3.0) curvature_gain = 3.0;
	double lookahead_distance = frc::SmartDashboard::GetNumber("AutoLookAhead", 0.3);
	if (lookahead_distance < 0.2) lookahead_distance = 0.2;
	if (lookahead_distance > 1.0) lookahead_distance = 1.0;


	DriveBase& drive_base = DriveBase::GetInstance();
	PurePursuitFollower* path_follower = new PurePursuitFollower(robot_path, &drive_base, new MechanismController2020, true);
	path_follower->GetFollowerParameters().m_kp = p_gain;
	path_follower->GetFollowerParameters().m_ki = i_gain;
	path_follower->GetFollowerParameters().m_kd = d_gain;
	path_follower->GetFollowerParameters().m_kv = 1.0/4.28;
	path_follower->GetFollowerParameters().m_kv_offset = 0.104; // 0.05719;
	path_follower->GetFollowerParameters().m_ka = 1.0/18.29;
	path_follower->GetFollowerParameters().m_wheelbase_width_m = 0.64;
	path_follower->GetFollowerParameters().m_period_s = 0.02;
	path_follower->GetFollowerParameters().m_max_velocity = max_velocity;
	path_follower->GetFollowerParameters().m_max_acceleration = max_acceleration;
	path_follower->GetFollowerParameters().m_curvature_gain = curvature_gain;
	path_follower->GetFollowerParameters().m_lookahead_distance = lookahead_distance;
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

//==============================================================================
// NOTE: These functions are broken, but left here until I ask what they are meant to be doing - Nick

// void DrivePathFollower::Rotate2Target(double max_velocity, double max_acceleration) {
// 	DriveBase& drive_base = DriveBase::GetInstance();

// 	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
// 	double rotateSpeed = 0.0;
// 	double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
	
// 	while (targetOffsetAngle_Horizontal > 1.0 || targetOffsetAngle_Horizontal < -1.0){
// 		targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
// 		if (targetOffsetAngle_Horizontal > 1.0)
// 		rotateSpeed = -0.5;
// 		if (targetOffsetAngle_Horizontal < -1.0)
// 		rotateSpeed = 0.5;
// 		drive_base.ArcadeDriveForVision(0.0, rotateSpeed);
		
// 	}
// }

// void DrivePathFollower::Rotate2Target2(double max_velocity, double max_acceleration) {

// 	// Modified from docs.limelightvision.io/en/latest/cs_aiming.html to fix errors

// 	double rotation = 0.0f;
// 	DriveBase& drive_base = DriveBase::GetInstance();
	
// 	//if (max_rotation > max_velocity)
// 	//	max_rotation = max_velocity;

// 	// Proportional constant Kp: Reduce from 1.0 until slight overshoot
// 	float kp = frc::SmartDashboard::GetNumber("VisionKp", 0.016);
// 	// Minimum rotation speed: Adjust so it is not quite enough to turn the robot
// 	// Test results: Use 0.25 wood or 0.33 for carpet tiles
// 	float minRotation = frc::SmartDashboard::GetNumber("VisionMinRotation", 0.25);
// 	// Maximum rotation speed - just to make sure we don't rotate too fast
// 	float maxRotation = frc::SmartDashboard::GetNumber("VisionMaxRotation", 1.0);

// 	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
// 	float tx = -table->GetNumber("tx", 0.0);  // degrees (-27 to 27 for limelight1)
// 	float tv = table->GetNumber("tv", 0.0);  // 0.0 unless the target is detected

// 	// Only turn if the target is detected
// 	if (tv != 0.0f) {

// 		// Give motors a little power even if error is small
// 		if (tx > 0.0)
// 			rotation = kp*tx + minRotation;
// 		else
// 			rotation = kp*tx - minRotation;

// 		// Clip the maximum rotation for safety!
// 		if (rotation > maxRotation)
// 			rotation = maxRotation;

// 		if (rotation < -maxRotation)
// 			rotation = -maxRotation;

// 		drive_base.ArcadeDriveForVision(0.0, rotation);
// 		// drive_base.Drive(0.0, rotation);
// 		// drive_base.Drive(0.0, 0.5);
// 		//rotation = 1.0;  // min = 0.2, max = 1.0?
// 		//drive_base.ArcadeDriveForVision(0.0, rotation);
// 	}
// }

// void DrivePathFollower::rotateForSkew(double max_velocity, double max_acceleration) {
// 	DriveBase& drive_base = DriveBase::GetInstance();

// 	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
// 	double rotateSpeed = 0.0;
// 	double targetOffsetSkew = table->GetNumber("ts",0.0);
// 	std::cout << "Offset Skew" << targetOffsetSkew << std::endl;
// 	while (targetOffsetSkew > -4.5 || targetOffsetSkew < -5.5){
// 		targetOffsetSkew = table->GetNumber("ts",0.0);
// 		if (targetOffsetSkew < -70 && table->GetNumber("tx", 0.0) > 2){
// 			targetOffsetSkew = targetOffsetSkew +90;
// 		}
// 		if (targetOffsetSkew > -4.5)
// 			rotateSpeed = -0.5;
// 		if (targetOffsetSkew < -6.5)
// 			rotateSpeed = 0.5;

// 		drive_base.ArcadeDriveForVision(0.0, rotateSpeed);
// 	}
// }
