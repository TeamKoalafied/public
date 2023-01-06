//==============================================================================
// TestFlashLed.cpp
//==============================================================================

#include "CycleLed.h"
#include "../Subsystems/Leds.h"



//==============================================================================
// Construction

CycleLed::CycleLed() {

}


//==============================================================================
// Function Overrides from frc::Command

void CycleLed::Initialize() {
    // Called just before this Command runs the first time

    // Turn on the required LED
	Leds::GetInstance().IncrementPattern();
}

void CycleLed::Execute() {
    // Called repeatedly when this Command is scheduled to run

    // Nothing to do. The LED just stays on.
}

bool CycleLed::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()
	return IsTimedOut();
}

void CycleLed::End() {
    // Called once after isFinished returns true

    // Turn off the LED

}

void CycleLed::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}
