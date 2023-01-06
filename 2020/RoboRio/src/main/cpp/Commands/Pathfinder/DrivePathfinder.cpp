//==============================================================================
// DrivePathfinder.cpp
//==============================================================================

#include "DrivePathfinder.h"
#include "PathController.h"
#include "JaciPathfinderController.h"
#include "KoalafiedPathfinderController.h"
#include "../MechanismController2020.h"
#include "../../Subsystems/DriveBase.h"
#include "../../Subsystems/Manipulator.h"

#include "Paths/RightTurn.h"
#include "Paths/RightSwitch.h"
#include "Paths/RightUTurn.h"
#include "Paths/RightUTurnSlow.h"
#include "Paths/Straight1.h"
#include "Paths/Straight3.h"

#include "../../RobotConfiguration.h"

#include <iostream>

namespace RC = RobotConfiguration;


//==========================================================================
// Static Member Variables

DrivePathfinder* DrivePathfinder::ms_test_command = NULL;


//==========================================================================
// Construction

DrivePathfinder::DrivePathfinder(PathController* path_controller):
	frc::Command("DrivePathfinder") {

	// Driving requires the DriveBase and Manipulator
    Requires(&DriveBase::GetInstance());
    Requires(&Manipulator::GetInstance());

    // Record the path controller to use
    m_path_controller = path_controller;
}

// Destructor
DrivePathfinder::~DrivePathfinder() {
	delete m_path_controller;
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
	bool own_trajectory = false;

	MechanismAction* mechanism_actions = NULL;
	int mechanism_actions_count = 0;

	int trajectory_length = 0;
	const char* name = NULL;

	static MechanismAction INTAKE_BALL[] = {
		 { "ExtendIntake", MechanismAction::TimeSpecification::Start, 1.0, 0 },
		 { "RunIndexForward", MechanismAction::TimeSpecification::Start, 1.0, 0 },
		 { "RetractIntake", MechanismAction::TimeSpecification::Start, 2.0, 0 },
	};

	int pov_angle = joystick->GetPOV(0);
	switch (pov_angle) {
		case RC::kJoystickPovUp:
			// 1m forward
			std::cout << "Starting DrivePathfinder - Straight1\n";
			left_trajectory = g_straight1_left_trajectory;
			right_trajectory = g_straight1_right_trajectory;
			trajectory_length = g_straight1_length;
			name = "Straight1";
			break;
		case RC::kJoystickPovLeft:
			// 3m forward and 2m right
			std::cout << "Starting DrivePathfinder - RightTurn\n";
			left_trajectory = g_right_turn_left_trajectory;
			right_trajectory = g_right_turn_right_trajectory;
			trajectory_length = g_right_turn_length;
			name = "RightTurn";
			break;
		case RC::kJoystickPovRight:
			// 3m forward and 1.4m to the right
			std::cout << "Starting DrivePathfinder - RightSwitch\n";
			left_trajectory = g_right_switch_left_trajectory;
			right_trajectory = g_right_switch_right_trajectory;
			trajectory_length = g_right_switch_length;
			name = "RightSwitch";
			break;
	}

	if (left_trajectory != NULL) {
		PathfinderController* controller = new JaciPathfinderController(left_trajectory, right_trajectory,
				                                                           trajectory_length, name, own_trajectory);
		//ms_test_command = new DrivePathfinder(new KoalafiedPathfinderController(left_trajectory, right_trajectory,

		controller->SetMechanismActions(new MechanismController2020, mechanism_actions, mechanism_actions_count);

		ms_test_command = new DrivePathfinder(controller);
		ms_test_command->Start();
	}

    if (joystick->GetRawButtonPressed(RobotConfiguration::kJoystickBButton)) {
		std::cout << "Reseting dead reckoning position\n";
		DriveBase& drive_base = DriveBase::GetInstance();
		drive_base.ResetPosition();
		drive_base.ResetPigeonHeading();
	}

}
