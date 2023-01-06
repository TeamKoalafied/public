//==============================================================================
// Indexer.cpp
//==============================================================================

#include "Indexer.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Indexer::Indexer()  {
    m_indexer_speed_controller = NULL;
}

Indexer::~Indexer() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Indexer::Setup() {
    std::cout << "Indexer::Setup()\n";
    
    // Create and configure the indexer Talon
    m_indexer_speed_controller = new TalonSRX(RobotConfiguration::kIndexerTalonId);
    m_kicker_speed_controller = new TalonSRX(RobotConfiguration::kKickerTalonId);
    TalonSRXConfiguration indexer_configuration;

    // Current limit
    indexer_configuration.continuousCurrentLimit = RobotConfiguration::kShooterMotorContinuousCurrentLimit;
    indexer_configuration.peakCurrentLimit = RobotConfiguration::kShooterMotorPeakCurrentLimit;
    indexer_configuration.peakCurrentDuration = RobotConfiguration::kShooterMotorPeakCurrentDurationMs;
    
    // Feedback sensor
    indexer_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;
    
    indexer_configuration.slot0.kF = 0.60;
    indexer_configuration.slot0.kP = 0.05;
    // Do all configuration and log if it fails
    int error = m_indexer_speed_controller->ConfigAllSettings(indexer_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the indexer Talon failed with code:  " << error << "\n";
    }
    error = m_kicker_speed_controller->ConfigAllSettings(indexer_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the kicker Talon failed with code:  " << error << "\n";
    }

    // Perform non-configuration setup
    m_indexer_speed_controller->SetSensorPhase(false); // Not reversed
    m_indexer_speed_controller->EnableCurrentLimit(true);
	m_indexer_speed_controller->SetNeutralMode(NeutralMode::Brake);
    m_kicker_speed_controller->SetSensorPhase(false); // Not reversed
    m_kicker_speed_controller->EnableCurrentLimit(true);
	m_kicker_speed_controller->SetNeutralMode(NeutralMode::Brake);
}

void Indexer::Shutdown() {
    std::cout << "Indexer::Shutdown()\n";
}

void Indexer::Periodic()
{
    frc::SmartDashboard::PutNumber("Indexer Current", m_indexer_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Indexer Output", m_indexer_speed_controller->GetMotorOutputPercent());         

    frc::SmartDashboard::PutNumber("Kicker Current", m_kicker_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Kicker Output", m_kicker_speed_controller->GetMotorOutputPercent());         
}


//==============================================================================
// Operations

void Indexer::ManualDriveIndexer(double index_percentage_output, double kicker_percentage_output) {
    m_indexer_speed_controller->Set(ControlMode::PercentOutput, -index_percentage_output);
    m_kicker_speed_controller->Set(ControlMode::PercentOutput, -kicker_percentage_output);
}

void Indexer::VelocityDriveIndexer(double percentage_speed){
    if (percentage_speed == 0) {
        m_indexer_speed_controller->Set(ControlMode::PercentOutput, 0);
    } else {
        const double MAX_RPM = 180.0; // TODO need to work this out properly
        double target_RPM = -percentage_speed * MAX_RPM;
        double target_native = KoalafiedUtilities::TalonSRXCtreVelocityRpmToNative(target_RPM);
        m_indexer_speed_controller->Set(ControlMode::Velocity, target_native);
    }
    
}
bool Indexer::HasHighCurrent() {
    return m_indexer_speed_controller->GetOutputCurrent() > 10.0;
}


void Indexer::TestDriveIndexer(frc::Joystick* joystick) {
    // Do tune driving of the indexer. Using the right Y for the drive and trigger
    // close loop with the left trigger button.
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);

    const double MAX_RPM = 180.0; // TODO need to work this out properly
    KoalafiedUtilities::TuneDriveTalonSRX(m_indexer_speed_controller, "Indexer", joystick_value, MAX_RPM, close_loop);
    KoalafiedUtilities::TuneDriveTalonSRX(m_kicker_speed_controller, "Kicker", joystick_value, MAX_RPM, close_loop);
}
