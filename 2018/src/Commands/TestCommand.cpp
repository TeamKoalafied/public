//==============================================================================
// TestCommand.cpp
//==============================================================================

#include <Commands/TestCommand.h>
#include "../Subsystems/TestSubsystem.h"
#include <iostream>


//==============================================================================
// Construction

TestCommand::TestCommand(Type type) :
    frc::Command("TestCommand") {
	m_type = type;

	m_pid_controller = new frc::PIDController(0.1, 0.0, 0.0, this, this, 0.02);
	m_pid_controller->SetInputRange(-10.0, 10.0);
	m_pid_controller->SetOutputRange(-1.0, 1.0);
	m_pid_controller->SetSetpoint(3.5);

    // This command requires the test subsystem
    Requires(&TestSubsystem::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void TestCommand::Initialize() {
    // Called just before this Command runs the first time

	switch (m_type) {
	case DoJoystick:
		std::cout << "TestCommand::Initialize() - DoJoystick\n";
		TestSubsystem::GetInstance().ResetDriving();
		break;
	case FlashLedForever:
		std::cout << "TestCommand::Initialize() - FlashLedForever\n";
		break;
	case PID:
		std::cout << "TestCommand::Initialize() - PID\n";
		m_pid_controller->Enable();
        m_periodic_timer.Init();
		break;
	}

	m_timer.Start();
}

void TestCommand::Execute() {
    // Called repeatedly when this Command is scheduled to run

	switch (m_type) {
	case DoJoystick:
		TestSubsystem::GetInstance().DoJoystick();
		break;
	case FlashLedForever: {
			int sequence = (int)(m_timer.Get() / 0.5);
			TestSubsystem::GetInstance().SetLedOn(0, sequence%2 == 0);
			break;
		}
	case PID:
		// Nothing
		break;
	}
}

bool TestCommand::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()
	return IsTimedOut();
}

void TestCommand::End() {
    // Called once after isFinished returns true
	switch (m_type) {
	case DoJoystick:
		std::cout << "TestCommand::End() - DoJoystick\n";
		break;
	case FlashLedForever:
		std::cout << "TestCommand::End() - FlashLedForever\n";
		break;
	case PID:
		std::cout << "TestCommand::End() - PID\n";
		m_pid_controller->Disable();
		break;
	}

}

void TestCommand::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}

//==========================================================================
// Function Overrides from frc::PIDSource

double TestCommand::PIDGet()
{
	std::cout << "TestCommand::PIDGet\n";
    // Print out information about how often the periodic function is being called
    m_periodic_timer.Periodic();
	return TestSubsystem::GetInstance().GetPosition();
//	return m_s.GetPosition();
//    return 1.0;
}

//==========================================================================
// Function Overrides from frc::PIDOutput

void TestCommand::PIDWrite(double output)
{
	std::cout << "TestCommand::PIDWrite " << output << "\n";
	TestSubsystem::GetInstance().Drive(output);
	//m_s.Drive(output);
}

