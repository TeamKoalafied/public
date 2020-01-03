//==============================================================================
// Zucc.cpp
//==============================================================================

#include "Zucc.h"

#include "frc/WPILib.h"

#include "../../RobotConfiguration.h"

#include <iostream>
#include <string>

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc/Ultrasonic.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Zucc::Zucc() {
  	m_continuous_power_solenoid = NULL;
  	m_zucc_direction_solenoid = NULL;
  	m_digital_output_zucc_suck = NULL;
  	m_digital_output_zucc_expel = NULL;
  	m_digital_input_zucc_indicator = NULL;
  	m_ultrasonic = NULL;
	m_expel_timer = NULL;
	m_expel_timer_running = false;
}

Zucc::~Zucc() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Zucc::Setup() {
    std::cout << "Zucc::Setup()\n";

    // Setup all solenoids - Vaccumn Power, Zucc Movement
	m_continuous_power_solenoid = new frc::Solenoid(RC::kPneumaticsControlModuleId,
													RC::kPneumaticsConstantPowerSolenoidId);
	m_continuous_power_solenoid->Set(true);
	m_zucc_direction_solenoid = new frc::Solenoid(RC::kPneumaticsControlModuleId,
												  RC::kPneumaticsZuccDirectionSolenoidId);

	// Setup all DIO
	m_digital_output_zucc_suck = new frc::DigitalOutput(RC::kDigitalOutZuccVacuumId);
	m_digital_output_zucc_expel = new frc::DigitalOutput(RC::kDigitalOutZuccReleaseId);
	m_digital_input_zucc_indicator = new frc::DigitalInput(RC::kDigitalInZuccSuctionIndicatorId);
	m_ultrasonic = new frc::Ultrasonic(RC::kUltrasonicTriggerPort, RC::kUltrasonicEchoPort);
	m_ultrasonic->SetAutomaticMode(true);

	m_expel_timer = new frc::Timer();
}

void Zucc::Shutdown() {
    std::cout << "Zucc::Shutdown()\n";
}

void Zucc::Periodic(bool show_dashboard) {
	switch (GetExpelTimerState()) {
		case ExpelTimerState::NotRunning:
		case ExpelTimerState::Running:
			// Do nothing
			break;
		case ExpelTimerState::Exceeded:
			// If the timer had been exceeded then turn off the expel. Note that this
			// should not normally be necessary, because one of StartVacuum(), HoldVacuum()
			// or StartExpel() will be called every time period.
			m_digital_output_zucc_expel->Set(false);
			break;
	}
	
	if (show_dashboard) {
		// Zucc control, status and soleniod state
		frc::SmartDashboard::PutBoolean("Zucc Vacuum", m_digital_output_zucc_suck->Get());
		frc::SmartDashboard::PutBoolean("Zucc Eject", m_digital_output_zucc_expel->Get());
		frc::SmartDashboard::PutBoolean("Zucc Ball", m_zucc_direction_solenoid->Get());
		frc::SmartDashboard::PutBoolean("Zucc Ultra", m_ultrasonic->GetRangeMM());
	}
	frc::SmartDashboard::PutBoolean("Zucc Detect", !m_digital_input_zucc_indicator->Get());
}

//==============================================================================
// Zucc Operation

void Zucc::StartVacuum() {
	// Start the vacuum and stop the expel
	m_digital_output_zucc_suck->Set(true);
	m_digital_output_zucc_expel->Set(false);

	// Reset the expel timer
	ResetExpelTimer();
}

void Zucc::HoldVacuum() {
	// If there is an object being held by the vacuum do nothing, otherwise
	// we stop things.
	if (!GetVacuumObjectDetected()) {
		// Stop the vacuum immediately
		m_digital_output_zucc_suck->Set(false);

		// Update the expel, depending on the current state of the timer
		switch (GetExpelTimerState()) {
		case ExpelTimerState::NotRunning:
			// Stop the expel. It should already be stopped, but just in case.
			m_digital_output_zucc_expel->Set(false);
			break;
		case ExpelTimerState::Running:
			// Do nothing. While holding we do not stop expelling, until the
			// timer has expired.
			break;
		case ExpelTimerState::Exceeded:
			// Stop the expel and reset the timer.
			m_digital_output_zucc_expel->Set(false);
			ResetExpelTimer();
			break;
		}
	}
}

void Zucc::StartExpel() {
	// Switch off the vacuum always
	m_digital_output_zucc_suck->Set(false);

	// Start the expel if it is not running
	switch (GetExpelTimerState()) {
		case ExpelTimerState::NotRunning:
			m_digital_output_zucc_expel->Set(true);
			StartExpelTimer();
			break;
		case ExpelTimerState::Running:
		case ExpelTimerState::Exceeded:
			// Do nothing. This function will only start the expel once, and will
			// not start it again until it has been reset.
			break;
	}
}

bool Zucc::GetVacuumObjectDetected() {
	// The zucc indicator is an inverted signal. 'false' means detected.
	return m_digital_output_zucc_suck->Get() && !m_digital_input_zucc_indicator->Get();
}

Zucc::Mode Zucc::GetMode()
{
	if (m_zucc_direction_solenoid->Get()) {
		return Mode::Ball;
	} else {
		return Mode::Hatch;
	}
}

void Zucc::SetBallMode() {
	if (GetVacuumObjectDetected()) {
		std::cout << "ERROR: Cannot change zucc mode when there is an object loaded\n";
		return;
	}
	m_zucc_direction_solenoid->Set(true);
}

void Zucc::SetHatchMode() {
	if (GetVacuumObjectDetected()) {
		std::cout << "ERROR: Cannot change zucc mode when there is an object loaded\n";
		return;
	}
	m_zucc_direction_solenoid->Set(false);
}

void Zucc::TestDriveZucc(frc::Joystick* joystick) {
	// Do completely manual zucc control for testing
	m_digital_output_zucc_suck->Set(joystick->GetRawButton(RC::kJoystickYButton));
	m_digital_output_zucc_expel->Set(joystick->GetRawButton(RC::kJoystickAButton));
	
	// Do standard manual zucc control
	// switch (joystick->GetPOV()) {
	// 	case RC::kJoystickPovUp:    StartVacuum(); break;
	// 	case RC::kJoystickPovDown:  StartExpel(); break;
	// 	case RC::kJoystickPovLeft:  SetBallMode(); break;
	// 	case RC::kJoystickPovRight: SetHatchMode(); break;
	// 	default:
	// 		HoldVacuum();
	// 		break;
	// }
}

//==========================================================================
// Expel Timer

void Zucc::StartExpelTimer()
{
	// Start the timer running from zero
	m_expel_timer->Reset();
	m_expel_timer->Start();
	m_expel_timer_running = true;
	std::cout << "Starting expel timer";
}

void Zucc::ResetExpelTimer()
{
	// Stop the timer if it is running
	if (m_expel_timer_running) {
		m_expel_timer->Stop();
		m_expel_timer_running = false;
		std::cout << "Stopping expel timer";
	}
}

Zucc::ExpelTimerState Zucc::GetExpelTimerState()
{
	if (m_expel_timer_running) {
		if (m_expel_timer->Get() > kExpelTimeS) {
			return ExpelTimerState::Exceeded;
		} else {
			return ExpelTimerState::Running;
		}

	} else {
		return ExpelTimerState::NotRunning;
	}
}
