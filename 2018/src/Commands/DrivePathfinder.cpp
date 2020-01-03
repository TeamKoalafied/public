//==============================================================================
// DrivePathfinder.cpp
//==============================================================================

#include "DrivePathfinder.h"
#include "PathController.h"
#include "Pathfinder/JaciPathfinderController.h"
#include "Pathfinder/KoalafiedPathfinderController.h"
#include "../Subsystems/DriveBase.h"

#include "Pathfinder/RightTurn.h"
#include "Pathfinder/RightSwitch.h"
#include "Pathfinder/RightUTurn.h"
#include "Pathfinder/RightUTurnSlow.h"
#include "Pathfinder/Straight1.h"
#include "Pathfinder/Straight3.h"

#include <iostream>


//==========================================================================
// Static Member Variables

DrivePathfinder* DrivePathfinder::ms_test_command = NULL;


//==========================================================================
// Construction

DrivePathfinder::DrivePathfinder(PathController* path_controller):
	frc::Command("DrivePathfinder") {

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());

    // Record the path controller to use
    m_path_controller = path_controller;
}


//==========================================================================
// Function Overrides from frc::Command

// Called just before this Command runs the first time
void DrivePathfinder::Initialize() {
	// Get the current encoder distances and heading then use them to initialise the Pathfinder trajectory
	DriveBase& drive_base = DriveBase::GetInstance();
	int left_encoder;
	int right_encoder;
	drive_base.GetEncoderDistances(left_encoder, right_encoder);
	double gyro_heading_deg = drive_base.GetPigeonHeading();
	m_path_controller->StartTrajectory(left_encoder, right_encoder, gyro_heading_deg);

	// Set the brake mode to true so we stop quickly if required
	drive_base.SetBrakeMode(true);
}

// Called repeatedly when this Command is scheduled to run
void DrivePathfinder::Execute() {
	// Get the current encoder distances
	DriveBase& drive_base = DriveBase::GetInstance();
	int left_encoder;
	int right_encoder;
	drive_base.GetEncoderDistances(left_encoder, right_encoder);

	// Get the robot heading
	double gyro_heading_deg = drive_base.GetPigeonHeading();

	// Get Pathfinder to calculate the required output to follow the trajectory
	double left_output;
	double right_output;
	m_path_controller->FollowTrajectory(left_encoder, right_encoder, gyro_heading_deg, left_output, right_output);

	// Drive the robot at the required speed
	drive_base.TankDriveOpenLoop(left_output, right_output);
}

// Make this return true when this Command no longer needs to run execute()
bool DrivePathfinder::IsFinished() {
    if (IsTimedOut()) {
        printf("DrivePathfinder command timed out\n");
        return true;
    }

    // The command is finished if the trajectory is finished
    return m_path_controller->IsTrajectoryFinished();
}

// Called once after isFinished returns true
void DrivePathfinder::End() {
	DriveBase& drive_base = DriveBase::GetInstance();
	drive_base.Stop();

	// Write the test data. We want this to happen even if the command is cancelled
	m_path_controller->WriteTestSampleToFile();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DrivePathfinder::Interrupted() {
	End();
}


//==========================================================================
// Static Joystick Testing Control Functions

void DrivePathfinder::DoJoystickTestControl(frc::Joystick* joystick)
{
	delete ms_test_command;
	ms_test_command = NULL;


	Segment* left_trajectory = NULL;
	Segment* right_trajectory = NULL;
	int trajectory_length = 0;
	const char* name = NULL;

	int pov_angle = joystick->GetPOV(0);
	switch (pov_angle) {
		case 0:
			std::cout << "Starting DrivePathfinder - Straight1\n";
			left_trajectory = g_straight1_left_trajectory;
			right_trajectory = g_straight1_right_trajectory;
			trajectory_length = g_straight1_length;
			name = "Straight1";
			break;
		case 90:
			std::cout << "Starting DrivePathfinder - RightTurn\n";
			left_trajectory = g_right_turn_left_trajectory;
			right_trajectory = g_right_turn_right_trajectory;
			trajectory_length = g_right_turn_length;
			name = "RightTurn";
			break;
		case 180:
			std::cout << "Starting DrivePathfinder - RightSwitch\n";
			left_trajectory = g_right_switch_left_trajectory;
			right_trajectory = g_right_switch_right_trajectory;
			trajectory_length = g_right_switch_length;
			name = "RightSwitch";
			break;
		case 270:
			std::cout << "Starting DrivePathfinder - RightUTurnSlow\n";
			left_trajectory = g_right_u_turn_slow_left_trajectory;
			right_trajectory = g_right_u_turn_slow_right_trajectory;
			trajectory_length = g_right_u_turn_slow_length;
			name = "RightUTurnSlow";
			break;
//		case 270:
//			std::cout << "Starting DrivePathfinder - RightUTurn\n";
//			left_trajectory = g_right_u_turn_left_trajectory;
//			right_trajectory = g_right_u_turn_right_trajectory;
//			trajectory_length = g_right_u_turn_length;
//			name = "RightUTurn";
//			break;
	}

	if (left_trajectory != NULL) {
		ms_test_command = new DrivePathfinder(new JaciPathfinderController(left_trajectory, right_trajectory,
				                                                           trajectory_length, name));
		//ms_test_command = new DrivePathfinder(new KoalafiedPathfinderController(left_trajectory, right_trajectory,
		//		                                                           trajectory_length, name));
		ms_test_command->Start();
	}
}

