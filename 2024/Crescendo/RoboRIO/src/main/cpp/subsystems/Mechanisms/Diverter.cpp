//==============================================================================
// Diverter.cpp
//==============================================================================
#include "Diverter.h"

#include <iostream>

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

namespace RC = RobotConfiguration;

Diverter::Diverter() {

}

Diverter::~Diverter() {

}

void Diverter::Setup() {
    m_diverter_controller = new TalonSRX(RC::kDiverterTalonId);
	TalonSRXConfiguration diverter_configuration;

    // Set the diverter current limits
	diverter_configuration.peakCurrentLimit = RC::kDiverterMotorPeakCurrentLimit;
	diverter_configuration.peakCurrentDuration = RC::kDiverterMotorPeakCurrentDurationMs;
	diverter_configuration.continuousCurrentLimit = RC::kDiverterMotorContinuousCurrentLimit;
    m_diverter_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.1.
	diverter_configuration.nominalOutputForward = +0.1;
	diverter_configuration.nominalOutputReverse = -0.1;
	diverter_configuration.peakOutputForward = +1.0;
	diverter_configuration.peakOutputReverse = -1.0;

    // Set the speed controller ramp rate
	diverter_configuration.openloopRamp = 0;
	diverter_configuration.closedloopRamp = RC::kDiverterMotorClosedLoopRampRateS;

	// PID parameters for Motion Magic control determined by velocity tuning
	diverter_configuration.slot0.kF = 0.5;
	diverter_configuration.slot0.kP = 0.5;
	diverter_configuration.slot0.kI = 0.0;
	diverter_configuration.slot0.kD = 0.0;

	// Do configuration
	int error = m_diverter_controller->ConfigAllSettings(diverter_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the diverter Talon failed with code:  " << error << "\n";
    }

    // The wrist should brake in neutral, otherwise it will fall slowly.
    m_diverter_controller->SetNeutralMode(NeutralMode::Brake);
}

void Diverter::Periodic() {

}

void Diverter::Shutdown() {

}

void Diverter::DivertToPath(bool amp_trap, double scale_factor) {
    double percentage_output = amp_trap ? scale_factor : -scale_factor; // Positive means trap/amp
    m_diverter_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Diverter::KickToShooter() {
	m_diverter_controller->Set(ControlMode::PercentOutput, -0.5);
}

void Diverter::ManualDriveDiverter(double percent_output) {
    m_diverter_controller->Set(ControlMode::PercentOutput, percent_output);
}

double Diverter::GetCurrent() const {
	return m_diverter_controller->GetStatorCurrent();
}

double Diverter::GetOutput() const {
	return m_diverter_controller->GetMotorOutputPercent();
}