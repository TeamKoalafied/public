//==============================================================================
// Intake.cpp
//==============================================================================

#include "Intake.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>


namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Intake::Intake()  {
    m_intake_speed_controller = NULL;
    m_intake_solenoid = NULL;
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

    // Current limit
    intake_configuration.continuousCurrentLimit = RobotConfiguration::kIntakeMotorContinuousCurrentLimit;
    intake_configuration.peakCurrentLimit = RobotConfiguration::kIntakeMotorPeakCurrentLimit;
    intake_configuration.peakCurrentDuration = RobotConfiguration::kIntakeMotorPeakCurrentDurationMs;

    // Use a fairly long ramp time so that the current doesn't existing the high current limit
    // and cause the intake to retract.
    intake_configuration.openloopRamp = 1;

    intake_configuration.slot0.kF = 0.12;
    intake_configuration.slot0.kP = 0.25;
    // Feedback sensor
    //intake_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;
 
    // Do all configuration and log if it fails
    int error = m_intake_speed_controller->ConfigAllSettings(intake_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }
    
    // Perform non-configuration setup
    //m_intake_speed_controller->SetSensorPhase(true); // Not reversed
    m_intake_speed_controller->EnableCurrentLimit(true);
	m_intake_speed_controller->SetNeutralMode(NeutralMode::Coast);

    // Initialise the intake solenoid
    m_intake_solenoid = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, RC::kPneumaticsIntakeSolenoidId);   
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic() {      
    frc::SmartDashboard::PutNumber("Intake Current", m_intake_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Intake Output", m_intake_speed_controller->GetMotorOutputPercent());         
}

//==============================================================================
// Operations


void Intake::Extend() {
    m_intake_solenoid->Set(true);
}

void Intake::Retract() {
    m_intake_solenoid->Set(false);
}

void Intake::Run() {
    // double target_velocity_rpm = max_rpm * drive;
	double target_velocity_native = KoalafiedUtilities::TalonSRXCtreVelocityRpmToNative(-1050);
    m_intake_speed_controller->Set(ControlMode::Velocity, target_velocity_native);
}

void Intake::RunReverse() {
    // double target_velocity_rpm = max_rpm * drive;
	double target_velocity_native = KoalafiedUtilities::TalonSRXCtreVelocityRpmToNative(600);
    m_intake_speed_controller->Set(ControlMode::Velocity, target_velocity_native);
}

void Intake::Stop() {
    ManualDriveIntake(0.0);
}

void Intake::ManualDriveIntake(double percentage_output) {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

bool Intake::HasHighCurrent() {
    return m_intake_speed_controller->GetOutputCurrent() > 12.5;
}

void Intake::TestDriveIntake(frc::Joystick* joystick) {
    // Test running the intake roller using the right X joystick
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    
    if (joystick->GetRawButton(RC::kJoystickXButton)) {
        Extend();
    }
    if (joystick->GetRawButton(RC::kJoystickBButton)) {
        Retract();
    }
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;

    m_intake_speed_controller->Set(ControlMode::PercentOutput, joystick_value);


//  NO CLOSED LOOP - there is no encoder on the Intake
//    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);
//    const double MAX_RPM = 1200.0; // TODO need to work this out properly
//    KoalafiedUtilities::TuneDriveTalonSRX(m_intake_speed_controller, "Intake", joystick_value, MAX_RPM, close_loop);
}
