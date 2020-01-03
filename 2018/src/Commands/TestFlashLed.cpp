//==============================================================================
// TestFlashLed.cpp
//==============================================================================

#include <Commands/TestFlashLed.h>
#include "../Subsystems/TestSubsystem.h"


//==============================================================================
// Construction

TestFlashLed::TestFlashLed(int led_index, double time_s) :
    frc::Command("FlashLed", time_s) {
	m_led_index = led_index;

    // This command requires the test subsystem
    Requires(&TestSubsystem::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void TestFlashLed::Initialize() {
    // Called just before this Command runs the first time

    // Turn on the required LED
	TestSubsystem::GetInstance().SetLedOn(m_led_index, true);

    printf("Turning LED %d on\n", m_led_index);
}

void TestFlashLed::Execute() {
    // Called repeatedly when this Command is scheduled to run

    // Nothing to do. The LED just stays on.
}

bool TestFlashLed::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()
	return IsTimedOut();
}

void TestFlashLed::End() {
    // Called once after isFinished returns true

    // Turn off the LED
	TestSubsystem::GetInstance().SetLedOn(m_led_index, false);
    printf("Turning LED %d off\n", m_led_index);
}

void TestFlashLed::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}
