//==============================================================================
// DriveWithJoystick.h
//==============================================================================

#ifndef DriveWithJoystick_H
#define DriveWithJoystick_H

#include <frc/commands/Command.h>



// Command to drive with the joystick. This is the default command for the drive base.
// This command run forever, until interupted, and simply tells the drive base to
// do joystick driving.
class DriveWithJoystick : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	DriveWithJoystick();

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================
};

#endif  // DriveWithJoystick_H
