//==============================================================================
// Intake.cpp
//==============================================================================

#include "Intake.h"

#include "../Manipulator.h"
#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

#include "../../Phoenix5Header.h"

namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Intake::Intake()  {
    m_intake_speed_controller = NULL;
}

Intake::~Intake() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Intake::Setup() {
    std::cout << "Intake::Setup()\n";

    // Create and configure the intake roller Talon
    m_intake_speed_controller = new TalonSRX(RobotConfiguration::kIntakeTalonId);
    TalonSRXConfiguration intake_configuration;

    // Current limit setup
    m_intake_speed_controller->EnableCurrentLimit(true);
    intake_configuration.continuousCurrentLimit = 25;
    intake_configuration.peakCurrentLimit = 20;
    intake_configuration.peakCurrentDuration = RobotConfiguration::kIntakeMotorPeakCurrentDurationMs;

    // Accelerate as hard as possible
    intake_configuration.openloopRamp = 0;

    // Do all configuration and log if it fails
    int error = m_intake_speed_controller->ConfigAllSettings(intake_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }

    // No need to stop the intake quickly, reduces strain on the belt
	m_intake_speed_controller->SetNeutralMode(NeutralMode::Coast);
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic() {      

}

double Intake::GetOutput() const {
    return m_intake_speed_controller->GetMotorOutputPercent();
}

double Intake::GetCurrent() const {
    return m_intake_speed_controller->GetStatorCurrent();
}

//==============================================================================
// Operations

void Intake::ManualDriveIntake(double percentage_output) {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

bool Intake::HasHighCurrent() {
    return m_intake_speed_controller->GetStatorCurrent() > 12.5;
}

void Intake::TestDriveIntake(frc::XboxController* joystick) {
    double roller_drive = frc::ApplyDeadband(joystick->GetLeftY(), RC::kJoystickDeadzone);
    m_intake_speed_controller->Set(ControlMode::PercentOutput, roller_drive);
}
