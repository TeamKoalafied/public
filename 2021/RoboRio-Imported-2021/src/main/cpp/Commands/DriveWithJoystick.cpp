//==============================================================================
// DriveWithJoystick.cpp
//==============================================================================

#include "DriveWithJoystick.h"

#include "../Subsystems/DriveBase.h"



//==============================================================================
// Construction

DriveWithJoystick::DriveWithJoystick() :
	frc::Command("DriveWithJoystick") {
	// Driving requires the drive base
    Requires(&DriveBase::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void DriveWithJoystick::Initialize() {
    // Called just before this Command runs the first time
    printf("DriveWithJoystick::Initialize()\n");

    // Reset the joystick state (prevents spurious button events)
    DriveBase::GetInstance().ResetJoystickState();
}

void DriveWithJoystick::Execute() {
    // Called repeatedly when this Command is scheduled to run

    // While this command is running just continually get the joystick and drive
    DriveBase::GetInstance().DoCheezyDrive();
}

bool DriveWithJoystick::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

    // Driving with the joystick never finishes, but it can be interrupted
	return false;
}

void DriveWithJoystick::End() {
    // Called once after isFinished returns true
    printf("DriveWithJoystick::End()\n");

    // When driving with the joystick ends, make sure the robot is stopped.
    DriveBase::GetInstance().Stop();
}

void DriveWithJoystick::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    printf("DriveWithJoystick::Interrupted()\n");

    // Call the base class, which will just end the command
    Command::Interrupted();
}
