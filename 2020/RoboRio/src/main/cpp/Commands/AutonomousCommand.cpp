//==============================================================================
// AutonomousCommand.cpp
//==============================================================================

#include "AutonomousCommand.h"

//#include "DriveStraight.h"
//#include "DriveRotate.h"
// #include "DrivePathfinder.h"
//#include "DrivePath.h"
#include "../Subsystems/DriveBase.h"
// #include "Pathfinder/LeftSwitchTrajectory.h"
// #include "Pathfinder/RightSwitchTrajectory.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>



//==============================================================================
// Static Data Members

frc::SendableChooser<AutonomousCommand::Strategy> AutonomousCommand::m_stratgey_chooser;
frc::SendableChooser<AutonomousCommand::Position> AutonomousCommand::m_position_chooser;

//==========================================================================
// Default dashboard data

// Build a single structure to keep all the mappings in one place to reduce risk of errors
struct AutoModeMapping {
	std::string label;
	AutonomousCommand::Position position;
	AutonomousCommand::Strategy strategy;
};

// Define the mapping between the labels on the dashboard and the position and strategy
const AutoModeMapping kAutoModes[] = {
//		{"Left Scale",                AutonomousCommand::Position::kLeft,    AutonomousCommand::Strategy::kLoadScale},
		{"Left   Straight",             AutonomousCommand::Position::kLeft,    AutonomousCommand::Strategy::kDriveForward},
		{"Middle Switch",             AutonomousCommand::Position::kMiddle,  AutonomousCommand::Strategy::kLoadSwitch},
		{"Right  Straight",            AutonomousCommand::Position::kRight,   AutonomousCommand::Strategy::kDriveForward},
//		{"Right Scale",               AutonomousCommand::Position::kRight,   AutonomousCommand::Strategy::kLoadScale},
		{"Left   PathScale",           AutonomousCommand::Position::kLeft,  AutonomousCommand::Strategy::kPathScale},
		{"Middle PathSwitch",           AutonomousCommand::Position::kMiddle,  AutonomousCommand::Strategy::kPathSwitch},
		{"Right  PathScale",           AutonomousCommand::Position::kRight,  AutonomousCommand::Strategy::kPathScale},
//		{"Middle PathfinderSwitch",   AutonomousCommand::Position::kMiddle,  AutonomousCommand::Strategy::kPathfinderSwitch},
//		{"Middle TestDriveAndReturn", AutonomousCommand::Position::kMiddle,  AutonomousCommand::Strategy::kTestDriveAndReturn}
};

// Compute the number of entries so we can create the string array for populating the default dashboard
static const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);


//==============================================================================
// Construction

AutonomousCommand::AutonomousCommand() :
    frc::CommandGroup("AutonomousCommand") {

	// Initialise to known values. These are all overwritten in SetupCommand(), but
	// initialising in the constructor is good practice and gets rid of warnings.
	m_strategy = Strategy::kDriveForward;
	m_position = Position::kLeft;
	m_position_inch = 0.0;
	m_game_data = "";
	m_our_switch_side = Side::kLeft;
	m_our_scale_side = Side::kLeft;
}


//==========================================================================
// Command Setup

bool AutonomousCommand::SetupCommand() {
	// Determine the strategy, robot position and the switch and scale sides
	int i;
	std::string result;

	// Get the strategy and the robot position from the dashboard.
	m_strategy = m_stratgey_chooser.GetSelected();
	m_position = m_position_chooser.GetSelected();

	// Attempt to get the autonomous strategy from the default dashboard . If nothing is
	// selected, then the returned string will not match and nothing will change.

	// Update network tables (not sure if this is needed, but it looked relevant)
	frc::SmartDashboard::UpdateValues();
	// Read the Auto Selector from the default dashboard
	result = frc::SmartDashboard::GetString("Auto Selector", "");
	// Search to see if one of the strings was selected
	for (i = 0; i < kAutoModeCount; i++) {
		if (kAutoModes[i].label == result) {
			m_position = kAutoModes[i].position;
			m_strategy = kAutoModes[i].strategy;
			std::cout << "Default dashboard mode \"" << kAutoModes[i].label << "\" selected\n";
		}
	}

	switch(m_position) {
	case Position::kLeft:   m_position_inch = kSideWallToAllianceInch + kRobotWidthInch/2; break;
	case Position::kMiddle: m_position_inch = kFieldWidthInch/2 - 12 + kRobotWidthInch/2; break;
	case Position::kRight:  m_position_inch = kFieldWidthInch - (kSideWallToAllianceInch + kRobotWidthInch/2); break;
	}

	// Get the game (switch/scale colours) from the driver station.
	m_game_data = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	// If the game data is not ready return false
	if (m_game_data.size() < 2) {
		std::cout << "AutonomousCommand::SetupCommand() failed. Game data was not ready.\n";
		return false;
	}

	// Log information about it input state that will determine the autonomous command
	std::cout << "==================================================================\n";
	std::cout << "AutonomousCommand Initialising\n";
	std::cout << "Strategy: ";
	switch (m_strategy) {
		case Strategy::kDriveForward: std::cout << "Drive Forward\n"; break;
		case Strategy::kLoadSwitch:   std::cout << "Load Switch\n";   break;
		case Strategy::kPathfinderSwitch:   std::cout << "Load Switch Pathfinder\n";   break;
		case Strategy::kLoadScale: std::cout << "Load Scale\n"; break;
		case Strategy::kPathSwitch: std::cout << "Load Switch Simple Path\n"; break;
		case Strategy::kPathScale: std::cout << "Load Scale Simple Path\n"; break;
		case Strategy::kTestDriveAndReturn: std::cout << "TEST - Drive and Return\n"; break;
	}

	std::cout << "Position: " << m_position_inch << "inches from the left of the alliance wall\n";
	std::cout << "Game Data: " << m_game_data << "\n";

	// Parse the game data and determine the which side of the switch and scale belong to us
	if (m_game_data.size() >= 2) {
		m_our_switch_side = m_game_data[0] == 'L' ? Side::kLeft : Side::kRight;
		m_our_scale_side = m_game_data[1] == 'L' ? Side::kLeft : Side::kRight;
	} else {
		std::cout << "ERROR: Game data not set. Assuming LLL.\n";
		m_our_switch_side = Side::kLeft;
		m_our_scale_side = Side::kLeft;
	}


	frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	std::cout << "Alliance: ";
	switch (alliance) {
		case frc::DriverStation::kRed:     std::cout << "Red\n"; break;
		case frc::DriverStation::kBlue:    std::cout << "Blue\n"; break;
		case frc::DriverStation::kInvalid: std::cout << "Invalid\n"; break;
	}

	// Reset the Pigeon heading so that we can make turns from a starting angle of 0
	DriveBase::GetInstance().ClearState();

	// Initialise this command based on the strategy
	switch (m_strategy) {
		case Strategy::kDriveForward: InitialiseDriveForward(); break;
		case Strategy::kLoadSwitch:   InitialiseLoadSwitch();   break;
		case Strategy::kPathfinderSwitch:   break; // InitialiseLoadSwitchPathfinder();   
		case Strategy::kLoadScale: InitialiseLoadScale(); break;
		case Strategy::kTestDriveAndReturn: InitialiseTestDriveAndReturn(); break;
		case Strategy::kPathSwitch: InitialiseLoadSwitchPath(); break;
		case Strategy::kPathScale: InitialiseLoadScalePath(); break;
	}

	// Setup was successful
	return true;
}


//==========================================================================
// Dashboard Setup

void AutonomousCommand::SetupDashboard() { // static
	// Setup the chooser for determining the strategy for the autonomous period
	//m_stratgey_chooser.
	m_stratgey_chooser.AddOption("Drive Forward", Strategy::kDriveForward);
	m_stratgey_chooser.SetDefaultOption("Load Switch", Strategy::kLoadSwitch);
	m_stratgey_chooser.AddOption("Load Switch Simple Path", Strategy::kPathSwitch);
	m_stratgey_chooser.AddOption("Load Switch PathFinder", Strategy::kPathfinderSwitch);
	m_stratgey_chooser.AddOption("Load Scale", Strategy::kLoadScale);
	m_stratgey_chooser.AddOption("Load Scale Simple Path", Strategy::kPathScale);
	m_stratgey_chooser.AddOption("Test Drive and Return", Strategy::kTestDriveAndReturn);

	frc::SmartDashboard::PutData("Autonomous Strategy", &m_stratgey_chooser);

	m_position_chooser.AddOption("Left", Position::kLeft);
	m_position_chooser.AddOption("Right", Position::kRight);
	m_position_chooser.SetDefaultOption("Middle", Position::kMiddle);
	frc::SmartDashboard::PutData("Starting Position", &m_position_chooser);

	// Also copy the autonomous mode options to the selector in the default dashboard.
	// We need a string array for the default dashboard, so we make it from the mode table
	unsigned int i;
	std::string auto_mode_labels[kAutoModeCount];
	for (i = 0; i < kAutoModeCount; i++)
		auto_mode_labels[i] = kAutoModes[i].label;

//	SmartDashboard::init();
	frc::SmartDashboard::PutStringArray("Auto List", auto_mode_labels);
	frc::SmartDashboard::UpdateValues();

}

void AutonomousCommand::EchoSettingsToDashboard() { // static
	std::string message = "";
	std::string result;

	switch (m_stratgey_chooser.GetSelected()) {
		case Strategy::kDriveForward:       message += "Drive Forward"; break;
		case Strategy::kLoadSwitch:   		message += "Load Switch";   break;
		case Strategy::kPathfinderSwitch:	message += "Load Switch Pathfinder";   break;
		case Strategy::kLoadScale:          message += "Load Scale"; break;
		case Strategy::kTestDriveAndReturn: message += "TEST - Drive and Return"; break;
		case Strategy::kPathSwitch:         message += "Load Switch Simple Path";   break;
		case Strategy::kPathScale:          message += "Load Scale Simple Path";   break;
	}


	switch (m_position_chooser.GetSelected()) {
		case Position::kLeft:   message += " - Left"; break;
		case Position::kMiddle: message += " - Middle"; break;
		case Position::kRight:  message += " - Right";break;
	}

	result = frc::SmartDashboard::GetString("Auto Selector", "");
	if ("Select Autonomous ..." != result)
		message += " - Mode overridden to: \"" + result + "\"";

	frc::SmartDashboard::PutString("Auto Settings", message);

	// Echo the default dashboard selection back to the default dashboard as a sanity check
	frc::SmartDashboard::PutString("DB/String 0", result);
}


//==========================================================================
// Strategy Initialisation

void AutonomousCommand::InitialiseDriveForward() {
	// The drive forward strategy only requires driving straight for a set distance

	// Go at a reasonable speed, but not too fast
	// const double kSpeedFeetPerS = 5.0;

	// Drive forward far enough to cross the line, but without hitting the switch (too hard)
	// We choose to go a position 75% of the distance from the auto line to the switch.
	// const double kDistanceToLine = kWallToAutoLineInch - kRobotLengthInch;
	// const double kDistanceToSwitch = kWallToSwitchInch - kRobotLengthInch;
	// double distance_inch = Lerp(kDistanceToLine, kDistanceToSwitch, 0.75);

	// Drive straight for the calculated distance on a heading of 0 (straight down the field)
	std::cout << "AutonomousCommand::InitialiseDriveForward()\n";
//    AddSequential(new DriveStraight(kSpeedFeetPerS, distance_inch, 0.0));
}

void AutonomousCommand::InitialiseLoadSwitch() {
	// The load switch strategy consists of driving to a position that is forward and
	// to the left or right depending on the initial robot position and the switch side
	// allocation.

	// Go at a reasonable speed, but not too fast
	// const double kDriveSpeedFeetPerS = 5.0;

	// Calculate the distance to the switch. We we drive half this distance, then to
	// the left/right adjustment, then drive the reset of the way.
	// const double distance_to_switch_inch = kWallToSwitchInch - kRobotLengthInch;

	// When loading the switch we want to be in half a robot distance from the
	// edge of the switch, plus a margin of error.
	// Calculate the distance to the load point from the wall on whichever side it
	// will be.
	double kMarginOfErrorInch = 4.0;
	double switch_load_position_from_wall_inch = kSideWallToSwitchInch + kRobotWidthInch/2.0 + kMarginOfErrorInch;

	// Calculate the distance across the field from the left wall to the point that we want the
	// centre of the robot to be at when loading the switch.
	double switch_load_position_from_left_inch = 0.0;
	switch (m_our_switch_side) {
		case Side::kLeft:
			switch_load_position_from_left_inch = switch_load_position_from_wall_inch;
			break;
		case Side::kRight:
			switch_load_position_from_left_inch = kFieldWidthInch - switch_load_position_from_wall_inch;
			break;
	}

	// Calculate the distance the robot has to move left or right. -ve is to the left.
	double switch_load_position_from_robot_start_inch = switch_load_position_from_left_inch - m_position_inch;

	// Distance to back away from the switch before lowering the lift
	// const double kBackUpDistanceInch = 24.0;

	// Build the command sequence to load the switch
	//
	std::cout << "AutonomousCommand::InitialiseLoadSwitch()\n";
	//AddSequential(new DriveStraight(kDriveSpeedFeetPerS, distance_to_switch_inch/2.0, 0.0));
	if (switch_load_position_from_robot_start_inch < 0.0) {
		// Need to drive left
		//AddSequential(new DriveRotate(90.0));
		//AddSequential(new DriveStraight(kDriveSpeedFeetPerS, -switch_load_position_from_robot_start_inch, 90.0));
		//AddSequential(new DriveRotate(-90.0));
	} else {
		// Need to drive right
		//AddSequential(new DriveRotate(-90.0));
		//AddSequential(new DriveStraight(kDriveSpeedFeetPerS, switch_load_position_from_robot_start_inch, -90.0));
		//AddSequential(new DriveRotate(90.0));
	}
	//AddSequential(new DriveElevator(Elevator::LiftPosition::kSwitch, false)); // Don't wait for the lift
	//AddSequential(new DriveStraight(kDriveSpeedFeetPerS, distance_to_switch_inch/2.0, 0.0));
	//AddSequential(new DriveArm(false));
	//AddSequential(new EjectClaw());
	//AddSequential(new DriveStraight(-kDriveSpeedFeetPerS, -kBackUpDistanceInch, 0.0));
	//AddSequential(new DriveArm(true));
	//AddSequential(new DriveElevator(Elevator::LiftPosition::kFloor, true)); // Do wait for the lift to get to the floor
}

/* void AutonomousCommand::InitialiseLoadSwitchPathfinder()
{
	DrivePathfinder* drive_command = new DrivePathfinder();
	PathfinderController& pathfinder_controller = drive_command->GetPathfinderController();
	switch (m_our_switch_side) {
		case Side::kLeft:
			pathfinder_controller.InitialiseTrajectoryFromArrays(g_left_switch_left_trajectory,
					g_left_switch_right_trajectory, g_left_switch_trajectory_length);
			break;
		case Side::kRight:
			pathfinder_controller.InitialiseTrajectoryFromArrays(g_right_switch_left_trajectory,
					g_right_switch_right_trajectory, g_right_switch_trajectory_length);
			break;
	}

	AddSequential(drive_command);

} */

void AutonomousCommand::InitialiseLoadScale(){
	// The load scale strategy consists of driving to a position that is forward and
		// to the left or right depending on the initial robot position and the scale side
		// allocation.


		// const double kDriveSpeedFeetPerS = 5.0;

		// Calculate the distance to the switch. We  drive half this distance, then to
		// the left/right adjustment, then continue to the scale.
		// const double distance_to_switch_inch = kWallToSwitchInch - kRobotLengthInch;
		// const double distance_to_scale_inch = kWallToScaleInch - kRobotLengthInch;
		// When loading the scale we use an adjustable variable to determine distance from the
		// edge of the scale.
		// Calculate the distance to the load point from the wall on whichever side it
		// will be.
		// double scale_robot_gap_inch = 10;


		// Calculate the distance across the field from the left wall to the point that we want the
		// centre of the robot to be at when passing the switch.
		// double switch_pass_position_from_left_inch = 0.0;
		// switch (m_our_scale_side) {
		// 	case Side::kLeft:
		// 		switch_pass_position_from_left_inch = kSideWallToSwitchInch/2.0;
		// 		break;
		// 	case Side::kRight:
		// 		switch_pass_position_from_left_inch = (kFieldWidthInch - kSideWallToSwitchInch)/2.0;
		// 		break;
		// }


		// double switch_pass_position_from_robot_star_inch = switch_pass_position_from_left_inch - m_position_inch;

		// std::cout << "AutonomousCommand::InitialiseLoadScale()\n";
		// AddSequential(new DriveStraight(kDriveSpeedFeetPerS, distance_to_switch_inch/2.0, 0.0));
		// if (switch_pass_position_from_robot_star_inch < 0.0) {
		// 	AddSequential(new DriveRotate(-90.0));
		// 	AddSequential(new DriveStraight(kDriveSpeedFeetPerS, -switch_pass_position_from_robot_star_inch, -90.0));
		// 	AddSequential(new DriveRotate(90.0));
		// } else {
		// 	AddSequential(new DriveRotate(90.0));
		// 	AddSequential(new DriveStraight(kDriveSpeedFeetPerS, switch_pass_position_from_robot_star_inch, 90.0));
		// 	AddSequential(new DriveRotate(-90.0));
		// }
		// AddSequential(new DriveStraight(kDriveSpeedFeetPerS, distance_to_scale_inch - distance_to_switch_inch/2.0 + kScalePlateWidthInch, 0.0));
		// AddSequential(new DriveRotate((m_our_scale_side == Side::kLeft)? 90:-90));
		// //AddSequential(new DriveElevator(Elevator::LiftPosition::kScale, true));
		// AddSequential(new DriveStraight(kDriveSpeedFeetPerS, kSideWallToScalePlatformInch - scale_robot_gap_inch, 0.0));
		//AddSequential(new DriveArm(false));
		//AddSequential(new EjectClaw());
}

void AutonomousCommand::InitialiseTestDriveAndReturn() {
	// const double kDriveSpeedFeetPerS = 5.0;
	// const double kDriveDistanceInch = 72.0;

	// This test command consists of the following steps:
	//   1. Drive forward
	//   2. Turn around 180 degrees
	//   3. Drive forward the same distance (should return to the starting position)
	//   4. Turn around 180 degrees (in the opposite direction for variety)
	std::cout << "AutonomousCommand::InitialiseTestDriveAndReturn()\n";
    // AddSequential(new DriveStraight(kDriveSpeedFeetPerS, kDriveDistanceInch, 0.0));
    // AddSequential(new DriveRotate(180.0));
    // AddSequential(new DriveStraight(kDriveSpeedFeetPerS, kDriveDistanceInch, 180.0));
    // AddSequential(new DriveRotate(-180.0));
}

void AutonomousCommand::InitialiseLoadSwitchPath() {
	// Load the correct switch side from the specified start side using the simple path following code

	// switch (m_position) {
	// 	case Position::kMiddle:
	// 		switch (m_our_switch_side) {
	// 			case Side::kLeft: AddSequential(new DrivePath(DrivePath::PathStrategy::kSwitchLeft));  break;
	// 			case Side::kRight: AddSequential(new DrivePath(DrivePath::PathStrategy::kSwitchRight)); break;
	// 			default: break;
	// 		}
	// 		break;
	// 	default: break;
	// }
}

void AutonomousCommand::InitialiseLoadScalePath() {
	// Load the correct scale side from the specified start side using the simple path following code

	// switch (m_position) {
	// 	case Position::kLeft:
	// 		switch (m_our_scale_side) {
	// 			case Side::kLeft:  AddSequential(new DrivePath(DrivePath::PathStrategy::kLeftScaleLeft));  break;
	// 			case Side::kRight: AddSequential(new DrivePath(DrivePath::PathStrategy::kLeftScaleRight)); break;
	// 			default: break;
	// 		}
	// 		break;
	// 	case Position::kRight:
	// 		switch (m_our_scale_side) {
	// 			case Side::kLeft:  AddSequential(new DrivePath(DrivePath::PathStrategy::kRightScaleLeft));  break;
	// 			case Side::kRight: AddSequential(new DrivePath(DrivePath::PathStrategy::kRightScaleRight)); break;
	// 			default: break;
	// 		}
	// 		break;
	// 	default: break;
	// }
}

//==========================================================================
// Utility Functions

double AutonomousCommand::Lerp(double start, double end, double ratio) {
	return start * (1.0 - ratio) + end * ratio;
}


