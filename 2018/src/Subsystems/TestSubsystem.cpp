//==============================================================================
// TestSubsystem.cpp
//==============================================================================

#include "TestSubsystem.h"

#include "../RobotConfiguration.h"
#include "../Utilities.h"
#include "../Commands/TestCommand.h"
#include "../Commands/TestPIDCommand.h"

#include <DriverStation.h>
#include <Joystick.h>
#include <SmartDashboard/SmartDashboard.h>
#include <iostream>

//==============================================================================
// Construction

TestSubsystem::TestSubsystem() :
    TSingleton<TestSubsystem>(this),
    frc::Subsystem("TestSubsystem"),
	m_pid_output(*this),
	m_pid_source(*this)
{

    for (int i = 0; i < kTotalLeds; i++) {
        m_led_digital_outputs[i] = NULL;
    }
	m_ultrasonic = NULL;
}

TestSubsystem::~TestSubsystem() {
    Shutdown();
}

//==============================================================================
// frc::Subsystem Function Overrides

void TestSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new TestCommand(TestCommand::DoJoystick));
}

void TestSubsystem::Periodic() {
	// Display the current joystick positions
	DisplayJoystickPosition();

	// If either of the trigger buttons is down start the default command.
	if (m_joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton) ||
		m_joystick->GetRawButton(RobotConfiguration::kJoystickRTrigButton)) {
		if (!GetDefaultCommand()->IsRunning()) {
			// Clear the joystick state, otherwise press and release events that
			// have already occurred my be incorrectly reported.
			KoalafiedUtilities::ClearJoystickButtonState(m_joystick);
			GetDefaultCommand()->Start();
		}
	}

	SmartDashboard::PutNumber("Left Trigger", m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftTriggerAxis));
	SmartDashboard::PutNumber("Right Trigger", m_joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis));
}


//==============================================================================
// Setup and Shutdown

void TestSubsystem::Setup() {
	// Create the digital outputs for the LEDs
    for (int i = 0; i < kTotalLeds; i++) {
        m_led_digital_outputs[i] = new frc::DigitalOutput(i);
    }

	// Create the ultrasonic sensor
	m_ultrasonic = new frc::Ultrasonic(3, 4);
	m_ultrasonic->SetAutomaticMode(true);

	// Create the joystick
    m_joystick = new frc::Joystick(0);

    // Display the joystick positions here so they will be available before enbling the robot
    DisplayJoystickPosition();

	// Setup the chooser for determining the testing mode
    m_mode_chooser.AddObject("Normal", Mode::kNormal);
    m_mode_chooser.AddDefault("Flood Test", Mode::kFloodTest);
	SmartDashboard::PutData("Test Mode", &m_mode_chooser);
}

void TestSubsystem::Shutdown() {
    for (int i = 0; i < kTotalLeds; i++) {
        delete m_led_digital_outputs[i];
        m_led_digital_outputs[i] = NULL;
    }
}

//==============================================================================
// Operation

bool TestSubsystem::GetLedOn(int number) {
    return m_led_digital_outputs[number]->Get();
}

void TestSubsystem::SetLedOn(int number, bool on) {
    m_led_digital_outputs[number]->Set(on);
}

void TestSubsystem::DoJoystick()
{
	// If the right trigger is pulled flood the log with messages. This is for
	// test if high log output causes instability.
	const int kMaxFloodLogsPerSec = 1000;
	const int kPeriodsPerSec = 50;
	int log_flood_per_sec = (int)(m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftTriggerAxis) *
							      kMaxFloodLogsPerSec);
	int log_flood_per_period = log_flood_per_sec / kPeriodsPerSec;
	int period_index = m_log_flood_counter++ % kPeriodsPerSec;
	int log_flood_remainder = log_flood_per_sec - log_flood_per_period*kPeriodsPerSec;
	if (period_index < log_flood_remainder) log_flood_per_period++;

	for (int i = 0; i < log_flood_per_period; i++) {
		std::cout << log_flood_per_sec << "  LOG FLOOD LOG FLOOD LOG FLOOD LOG FLOOD LOG FLOOD LOG FLOOD LOG FLOOD LOG FLOOD\n";
	}


	SmartDashboard::PutNumber("Range (inch)", m_ultrasonic->GetRangeInches());

	// Log if the A button is 'pressed', 'released', or just down. This shows how the
	// joystick button state works.
	if (m_joystick->GetRawButtonPressed(RobotConfiguration::kJoystickAButton)) {
		std::cout << "Joystick A button pressed\n";

		Mode mode = m_mode_chooser.GetSelected();
		switch (mode) {
			case Mode::kNormal:    std::cout << "Mode: Normal\n";     break;
			case Mode::kFloodTest: std::cout << "Mode: Flood Test\n"; break;
			default:               std::cout << "Mode: default\n";    break;
		}

	}
	if (m_joystick->GetRawButtonReleased(RobotConfiguration::kJoystickAButton)) {
		std::cout << "Joystick A button released\n";
	}
	if (m_joystick->GetRawButton(RobotConfiguration::kJoystickAButton)) {
		std::cout << "Joystick A button down\n";
	}

	double drive = m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis);
	if (fabs(drive) < RobotConfiguration::kJoystickDeadzone) drive = 0.0;
	Drive(drive);

	if (m_joystick->GetRawButtonPressed(RobotConfiguration::kJoystickBButton)) {
//		(new TestPIDCommand(20))->Start();

		if (m_pid_controller.IsEnabled()) {
			std::cout << "Disabling PIDController\n";
			m_pid_controller.Disable();
		} else {
			std::cout << "Enabling PIDController\n";
			ResetDriving();
			m_pid_controller.SetPID(0.1, 0.0, 0.0);
			m_pid_controller.SetInputRange(-10.0, 10.0);
			m_pid_controller.SetOutputRange(-1.0, 1.0);
			m_pid_controller.SetSetpoint(3.5);
			m_pid_controller.Enable();
		}
	}

	if (m_joystick->GetRawButtonPressed(RobotConfiguration::kJoystickXButton)) {
		(new TestCommand(TestCommand::PID))->Start();
	}
}

//==========================================================================
// Fake Driving

void TestSubsystem::ResetDriving()
{
	m_position = 0.0;
	m_velocity = 0.0;
	m_acceleration = 0.0;
	m_drive_timer.Reset();
	m_drive_timer.Start();

}

void TestSubsystem::Drive(double drive_value)
{
	UpdateFakeDriving();
	m_acceleration = 10.0 * drive_value;
}

double TestSubsystem::GetPosition()
{
	UpdateFakeDriving();
	return m_position;
}

//==========================================================================
//

void TestSubsystem::UpdateFakeDriving()
{
	double delta_t = m_drive_timer.Get();
	m_drive_timer.Reset();


	double actual_acceleration = m_acceleration - m_velocity/1.0;


	double initial_velocity = m_velocity;
	m_velocity += actual_acceleration * delta_t;

	m_position += initial_velocity*delta_t + 0.5*actual_acceleration*delta_t*delta_t;

	SmartDashboard::PutNumber("Acceleration", m_acceleration);
	SmartDashboard::PutNumber("Velocity", m_velocity);
	SmartDashboard::PutNumber("Position", m_position);
}


//==========================================================================
//

void TestSubsystem::DisplayJoystickPosition()
{
	SmartDashboard::PutNumber("Trigger Left", m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftTriggerAxis));
	SmartDashboard::PutNumber("Trigger Right", m_joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis));
	SmartDashboard::PutNumber("Left Joystick X", m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftXAxis));
}


//==========================================================================

TestSubsystem::MyPIDOutput::MyPIDOutput(TestSubsystem& s) :
		m_s(s) {
}

void TestSubsystem::MyPIDOutput::PIDWrite(double output) {
	std::cout << "MyPIDOutput::PIDWrite " << output << "\n";
	m_s.Drive(output);
}

TestSubsystem::MyPIDSource::MyPIDSource(TestSubsystem& s) :
		m_s(s) {
}

double TestSubsystem::MyPIDSource::PIDGet() {
	std::cout << "MyPIDSource::PIDGet\n";
	return m_s.GetPosition();
}
