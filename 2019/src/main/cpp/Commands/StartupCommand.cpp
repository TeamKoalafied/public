//==============================================================================
// StartupCommand.cpp
//==============================================================================

#include "StartupCommand.h"

#include "../Subsystems/Manipulator.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace frc;


//==============================================================================
// Construction

StartupCommand::StartupCommand() :
    frc::Command("StartupCommand", 14.0) { // Timeout after 5.0s

    m_timer.Start();
    m_execute_count = 0;

    Requires(&Manipulator::GetInstance());
}

//==============================================================================
// Function Overrides from frc::Command

void StartupCommand::Initialize() {
    // Called just before this Command runs the first time
    printf("StartupCommand::Initialize() %fms\n", m_timer.Get()*1000.0);

    // Move the pivot to the 'vertical' position. This means that the hatch is
    // less likly to fall off.
    Manipulator::GetInstance().GotoVertical();
}

void StartupCommand::Execute() {
    // Called repeatedly when this Command is scheduled to run

    // While this command is running just sucks with the zucc.
	// NOTE: The fact that this command is running means that normal operator joystick is
	// not running and hence the zucc will not be rotated to the standard angle.
    Manipulator::GetInstance().StartVacuum();
    m_execute_count++;
}

bool StartupCommand::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

    // We are done when the hatch is detected by the vacuum
	return Manipulator::GetInstance().GetVacuumObjectDetected() || IsTimedOut();
}

void StartupCommand::End() {
    // Called once after isFinished returns true
    printf("StartupCommand::End() %fms count %d\n", m_timer.Get()*1000.0, m_execute_count);

    // Switch to holding with the vacuum
    Manipulator::GetInstance().HoldVacuum();
}

void StartupCommand::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    printf("StartupCommand::Interrupted()\n");

    // Call the base class, which will just end the command
    Command::Interrupted();
}
