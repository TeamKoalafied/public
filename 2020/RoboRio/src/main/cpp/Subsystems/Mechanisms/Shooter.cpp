//==============================================================================
// Shooter.cpp
//==============================================================================

#include "Shooter.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Shooter::Shooter()  {
}

Shooter::~Shooter() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Shooter::Setup() {
    std::cout << "Shooter::Setup()\n";
    
    m_shooter_master_speed_controller = new TalonFX(RobotConfiguration::kShooterMasterTalonId);
    m_shooter_slave_speed_controller = new TalonFX(RobotConfiguration::kShooterSlaveTalonId);

    m_shooter_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kShooterMasterTalonId);
    m_shooter_slave_speed_controller->SetInverted(true);

    TalonFXConfiguration shooter_configuration;

    shooter_configuration.nominalOutputReverse = -0.0f;
    shooter_configuration.nominalOutputForward = 0.0f;

    shooter_configuration.peakOutputReverse = -1.0f;
    shooter_configuration.peakOutputForward = +1.0f;

    // ramp rate needs to be longerto prevent massive voltage drop
    shooter_configuration.closedloopRamp = 1;
    shooter_configuration.openloopRamp = 1;

    shooter_configuration.forwardSoftLimitEnable = false;
    shooter_configuration.reverseSoftLimitEnable = false;

    shooter_configuration.closedloopRamp = 0.5;
    shooter_configuration.openloopRamp = 0.5;
    
    shooter_configuration.supplyCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration (true, 
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentDurationMs);

    shooter_configuration.slot0.kF = 0.052;
    shooter_configuration.slot0.kP= 0.32;
    
    // shooter_configuration.statorCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration ();
        
    int error = m_shooter_master_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the shooter Talon failed with code:  " << error << "\n";
    }

    int error2 = m_shooter_slave_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error2 != 0) {
        std::cout << "Configuration of the shooter slave Talon failed with code:  " << error2 << "\n";
    }
    // Comment out PID settings so they don't override the Phoenix tuner.
    // m_shooter_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
	m_shooter_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
    m_shooter_slave_speed_controller->SetNeutralMode(NeutralMode::Coast);

    m_shooter_master_speed_controller->SetSensorPhase(false);
}

void Shooter::Shutdown() {
    std::cout << "Shooter::Shutdown()\n";
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Current", m_shooter_master_speed_controller->GetOutputCurrent());            
    frc::SmartDashboard::PutNumber("Shooter Output", m_shooter_master_speed_controller->GetMotorOutputPercent());              
    frc::SmartDashboard::PutNumber("Shooter Speed RPM", GetShooterRpm());
}


//==============================================================================
// Operations

void Shooter::ManualDriveShooter(double percentage_speed) {
    m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

void Shooter::DriveShooterRpm(double shooter_speed_rpm) {
    double shooter_motor_speed_rpm = -shooter_speed_rpm / RC::kShooterMotorGearRatio;
    double shooter_speed_native =  KoalafiedUtilities::TalonFXVelocityRpmToNative(shooter_motor_speed_rpm);
    m_shooter_master_speed_controller->Set(ControlMode::Velocity, shooter_speed_native);
}

double Shooter::GetShooterRpm() {
    double shooter_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double shooter_motor_speed_rpm =  KoalafiedUtilities::TalonFXVelocityNativeToRpm(shooter_speed_native);
    double shooter_speed_rpm = shooter_motor_speed_rpm * RC::kShooterMotorGearRatio;
    return -shooter_speed_rpm;
}

bool Shooter::ShooterAtSpeed(double desired_shooter_speed_rpm) {
    double shooter_speed_rpm = GetShooterRpm();

    double diffpercent = fabs(desired_shooter_speed_rpm - shooter_speed_rpm) * 100 / desired_shooter_speed_rpm;
    return (diffpercent < 5);
}

void Shooter::TestDriveShooter(frc::Joystick* joystick) {

    // Do tune driving of the shooter. If the A button is down drive to the dashboard speed,
    // otherwise use the right Y for the drive and trigger close loop with the left trigger button.
    if (joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton)) {
        double shooter_speed_rpm = (frc::SmartDashboard::GetNumber("dRPM", 4000.0)) * -1;
        double shooter_motor_speed_rpm = shooter_speed_rpm / RC::kShooterMotorGearRatio;

        // Still use the test drive function as it records everything to the dashboard.
        KoalafiedUtilities::TuneDriveTalonFX(m_shooter_master_speed_controller, "Shooter", 1.0, shooter_motor_speed_rpm, true);
    }
    else {
        double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
        if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
        bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickRTrigButton);
        const double MAX_RPM = 4000.0;
        KoalafiedUtilities::TuneDriveTalonFX(m_shooter_master_speed_controller, "Shooter", joystick_value, MAX_RPM, close_loop);
    }
}
