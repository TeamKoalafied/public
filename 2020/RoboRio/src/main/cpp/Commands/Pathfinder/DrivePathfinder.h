//==============================================================================
// DrivePathfinder.h
//==============================================================================

#pragma once

#include <frc/commands/Command.h>
#include <frc/Joystick.h>
class PathController;



class DrivePathfinder : public frc::Command {
public:
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	DrivePathfinder(PathController* path_controller);

	// Destructor
	~DrivePathfinder();

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

	PathController* m_path_controller;		// The path control that drives the trajectory for this command

    static DrivePathfinder* ms_test_command;// Test command used by DoJoystickTestControl()
};

