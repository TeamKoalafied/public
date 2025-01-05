//==============================================================================
// Trampler.cpp
//==============================================================================
#include "Trampler.h"

#include <iostream>

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

namespace RC = RobotConfiguration;

Trampler::Trampler() {

}

Trampler::~Trampler() {

}

void Trampler::Setup() {
    m_trampler_controller = new TalonSRX(RC::kTramplerTalonId);
    TalonSRXConfiguration trampler_configuration;

    // Set the trampler current limits
	trampler_configuration.peakCurrentLimit = RC::kDiverterMotorPeakCurrentLimit;
	trampler_configuration.peakCurrentDuration = RC::kDiverterMotorPeakCurrentDurationMs;
	trampler_configuration.continuousCurrentLimit = RC::kDiverterMotorContinuousCurrentLimit;
    m_trampler_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.4.
	trampler_configuration.nominalOutputForward = +0.1;
	trampler_configuration.nominalOutputReverse = -0.1;
	trampler_configuration.peakOutputForward = +1.0;
	trampler_configuration.peakOutputReverse = -1.0;

    // Set the speed controller ramp rate
	trampler_configuration.openloopRamp = RC::kDiverterMotorOpenLoopRampRateS;
	trampler_configuration.closedloopRamp = RC::kDiverterMotorClosedLoopRampRateS;

	// PID parameters for Motion Magic control determined by velocity tuning
	trampler_configuration.slot0.kF = 0.5;
	trampler_configuration.slot0.kP = 0.5;
	trampler_configuration.slot0.kI = 0.0;
	trampler_configuration.slot0.kD = 0.0;

	// Do configuration
	int error = m_trampler_controller->ConfigAllSettings(trampler_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the diverter Talon failed with code:  " << error << "\n";
    }

    // The wrist should brake in neutral, otherwise it will fall slowly.
    m_trampler_controller->SetNeutralMode(NeutralMode::Brake);
}


void Trampler::Periodic() {

}

void Trampler::Shutdown() {
    
}

void Trampler::ManualDriveTrampler(double percent_output) {
    m_trampler_controller->Set(ControlMode::PercentOutput, -percent_output);

}

double Trampler::GetCurrent() const {
	return m_trampler_controller->GetStatorCurrent();
}

double Trampler::GetOutput() const {
	return m_trampler_controller->GetMotorOutputPercent();
}