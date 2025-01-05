//==============================================================================
// Winch.h
//==============================================================================

#include "Winch.h"

#include "../ManipulatorShuffleboard.h"
#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"


#include <iostream>

namespace RC = RobotConfiguration;

Winch::Winch() {

}

Winch::~Winch() {
    Shutdown();
}

void Winch::Setup() {
    m_winch_controller = new TalonSRX(RC::kWinchTalonId);

    TalonSRXConfiguration winch_configuration;
    
	// Set the wrist current limits
	winch_configuration.peakCurrentLimit = RC::kWinchMotorPeakCurrentLimit;
	winch_configuration.peakCurrentDuration = RC::kWinchMotorPeakCurrentDurationMs;
	winch_configuration.continuousCurrentLimit = RC::kWinchMotorContinuousCurrentLimit;
    m_winch_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.4.
	winch_configuration.nominalOutputForward = +0.1;
	winch_configuration.nominalOutputReverse = -0.1;
	winch_configuration.peakOutputForward = +1.0;
	winch_configuration.peakOutputReverse = -1.0;

    // Set the speed controller ramp rate
	winch_configuration.openloopRamp = 0.1;
	winch_configuration.closedloopRamp = 0.1;

	// Setup soft limits at extremes of the climbers reach
	m_winch_forward_limit = kForwardLimit;
	m_winch_reverse_limit = kReverseLimit;
	winch_configuration.forwardSoftLimitEnable = true;
	winch_configuration.forwardSoftLimitThreshold = m_winch_forward_limit / kWinchInchPerEncoder;
	winch_configuration.reverseSoftLimitEnable = true;
	winch_configuration.reverseSoftLimitThreshold = m_winch_reverse_limit / kWinchInchPerEncoder;

	// PID parameters for Position control 
	winch_configuration.slot0.kF = 0.00;
	winch_configuration.slot0.kP = 0.1;
	winch_configuration.slot0.kI = 0.0;
	winch_configuration.slot0.kD = 0.0;

    // Do configuration
	int error = m_winch_controller->ConfigAllSettings(winch_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the diverter Talon failed with code:  " << error << "\n";
    }

    // The wrist should brake in neutral, otherwise it will fall slowly.
    m_winch_controller->SetNeutralMode(NeutralMode::Brake);

	// Winch encode is reversed compared to the motor drive direction
	m_winch_controller->SetSensorPhase(true);

	// Reset the encoder when the code starts. The winch has no limit switches and so it relies
	// on being started at the zero position.
	m_winch_controller->SetSelectedSensorPosition(0);
}

void Winch::Periodic(ManipulatorShuffleboard* shuffleboard) {
	if (shuffleboard->GetWinchForwardLimit() != m_winch_forward_limit ||
		shuffleboard->GetWinchReverseLimit() != m_winch_reverse_limit) {
	
		// Set the wrist current limits
		TalonSRXConfiguration winch_configuration;
		m_winch_controller->GetAllConfigs(winch_configuration);

		// Setup soft limits at extremes of the climbers reach
		m_winch_forward_limit = shuffleboard->GetWinchForwardLimit();
		m_winch_reverse_limit = shuffleboard->GetWinchReverseLimit();
		winch_configuration.forwardSoftLimitEnable = true;
		winch_configuration.forwardSoftLimitThreshold = m_winch_forward_limit / kWinchInchPerEncoder;
		winch_configuration.reverseSoftLimitEnable = true;
		winch_configuration.reverseSoftLimitThreshold = m_winch_reverse_limit / kWinchInchPerEncoder;

		// Do configuration
		std::cout << "Reconfigurint winch limits to :  " << m_winch_forward_limit.value()  << " - " << m_winch_reverse_limit.value() << "\n";
		int error = m_winch_controller->ConfigAllSettings(winch_configuration, RC::kTalonTimeoutMs);
		if (error != 0) {
			std::cout << "Configuration of the diverter Talon failed with code:  " << error << "\n";
		}
	}
}

void Winch::Shutdown() {

}

void Winch::ManualDriveWinch(double percentage_output) {
    m_winch_controller->Set(ControlMode::PercentOutput, percentage_output);
	m_winch_extension_set_inch = kWinchExtensionNotSet;
}

void Winch::HoldPosition(units::inch_t position) {
	double winch_extension_native = position / kWinchInchPerEncoder;

	m_winch_extension_set_inch = position;

	m_winch_controller->Set(ControlMode::Position, winch_extension_native);
}

double Winch::GetOutput() const {
    return m_winch_controller->GetMotorOutputPercent();
}

double Winch::GetCurrent() const {
    return m_winch_controller->GetStatorCurrent();
}

units::inch_t Winch::GetWinchPosition() const {
	// Get the position in encoder units and convert to inches
	double extension_encoder = m_winch_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	units::inch_t extension_inch = extension_encoder * kWinchInchPerEncoder;
	return extension_inch;

}

bool Winch::IsExtensionSet() const {
	return (m_winch_extension_set_inch != kWinchExtensionNotSet);
}
