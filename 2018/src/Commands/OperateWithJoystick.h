//==============================================================================
// OperateWithJoystick.h
//==============================================================================

#ifndef OperateWithJoystick_H
#define OperateWithJoystick_H

#include <Commands/Command.h>

// Command to operate the elevator with the joystick. This is the default command
// for the elevator. This command run forever, until interupted, and simply tells
// the Elevator to do joystick operation.
class OperateWithJoystick : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	OperateWithJoystick();

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================
};

#endif  // OperateWithJoystick_H
