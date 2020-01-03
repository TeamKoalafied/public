//==============================================================================
// DrivePathfinder.h
//==============================================================================

#pragma once

#include <Commands/Command.h>
#include <Joystick.h>
class PathController;


class DrivePathfinder : public frc::Command {
public:
    //==========================================================================
    // Construction

	// Constuctor
	DrivePathfinder(PathController* path_controller);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

    //==========================================================================
	// Static Joystick Testing Control Functions

	// Do joystick control of following Pathfinder paths for testing
	//
	// joystick - The joystick to query for user input
	static void DoJoystickTestControl(frc::Joystick* joystick);

private:
    //==========================================================================
    // Member Variables

	PathController* m_path_controller;

    static DrivePathfinder* ms_test_command;
};

