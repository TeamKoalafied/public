//==============================================================================
// OperateWithJoystick.cpp
//==============================================================================

#include "OperateWithJoystick.h"

#include "../Subsystems/Elevator.h"


//==============================================================================
// Construction

OperateWithJoystick::OperateWithJoystick() :
	frc::Command("OperateWithJoystick") {
	// Driving requires the elevator
    Requires(&Elevator::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void OperateWithJoystick::Initialize() {
    // Called just before this Command runs the first time
    printf("OperateWithJoystick::Initialize()\n");

    // Reset the joystick state (prevents spurious button events)
    Elevator::GetInstance().ResetJoystickState();
}

void OperateWithJoystick::Execute() {
    // Called repeatedly when this Command is scheduled to run

    // While this command is running just continually drive the elevator with the joystick
	Elevator::GetInstance().DoOperatorDrive();
}

bool OperateWithJoystick::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

    // Operating with the joystick never finishes, but it can be interrupted
	return false;
}

void OperateWithJoystick::End() {
    // Called once after isFinished returns true
    printf("OperateWithJoystick::End()\n");

    // When driving with the joystick ends, make sure the elevator is stopped.
    Elevator::GetInstance().StopClawRollers();
}

void OperateWithJoystick::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    printf("OperateWithJoystick::Interrupted()\n");

    // Call the base class, which will just end the command
    Command::Interrupted();
}
