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

//const double Shooter::kShooterDegreesPerEncoder = 360.0 / (4096.0 * RC::kShooterRetractRatio);


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
    
    // Create the TalonFX controller for the shooter motors. The slave is inverted because
    // the two motors are attached to opposite ends of the shooter wheel shaft and hence
    // need to rotate in opposite directions.
    m_shooter_master_speed_controller = new TalonFX(RobotConfiguration::kShooterMasterTalonId);
//    m_shooter_slave_speed_controller = new TalonFX(RobotConfiguration::kShooterSlaveTalonId);
//    m_shooter_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kShooterMasterTalonId);
//    m_shooter_slave_speed_controller->SetInverted(true);

    // Configure the nominal and peak outputs to just cover the whole range
    TalonFXConfiguration shooter_configuration;
    shooter_configuration.nominalOutputReverse = -0.0f;
    shooter_configuration.nominalOutputForward = 0.0f;
    shooter_configuration.peakOutputReverse = -1.0f;
    shooter_configuration.peakOutputForward = +1.0f;

    // Ramp rate needs to be fairly long to prevent massive voltage drop as the
    // motor starts up
    shooter_configuration.closedloopRamp = 1;
    shooter_configuration.openloopRamp = 1;

    shooter_configuration.forwardSoftLimitEnable = false;
    shooter_configuration.reverseSoftLimitEnable = false;

    // Current limites
    shooter_configuration.supplyCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration (true, 
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentDurationMs);

    // Velocity PID parameters 
    shooter_configuration.slot0.kF = 0.05;
    shooter_configuration.slot0.kP = 0.2;
    shooter_configuration.slot0.kI = 0.0;
    shooter_configuration.slot0.kD = 0.0;
    
    // Configure the two motors        
    int error = m_shooter_master_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the shooter Talon failed with code:  " << error << "\n";
    }
//    int error2 = m_shooter_slave_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
//    if (error2 != 0) {
//        std::cout << "Configuration of the shooter slave Talon failed with code:  " << error2 << "\n";
//    }

    // Set the motors to coast. Braking is unnecessary and will cause huge back EMF
	m_shooter_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
//    m_shooter_slave_speed_controller->SetNeutralMode(NeutralMode::Coast);

    m_shooter_master_speed_controller->SetSensorPhase(false);
}

void Shooter::Shutdown() {
    std::cout << "Shooter::Shutdown()\n";
}

void Shooter::Periodic() {
    // Report some shooter parameters to the dashboard
    frc::SmartDashboard::PutNumber("Shooter Current", m_shooter_master_speed_controller->GetOutputCurrent());            
    frc::SmartDashboard::PutNumber("Shooter Output", m_shooter_master_speed_controller->GetMotorOutputPercent());              
    frc::SmartDashboard::PutNumber("Shooter Speed RPM", GetShooterRPM());
}


//==============================================================================
// Operations

void Shooter::DriveShooterOpenLoop(double percentage_speed) {
    m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

bool Shooter::DriveShooterClosedLoop(double shooter_wheel_rpm) {
    // Convert the shooter shaft speed to a motor speed using the gear ratio and clip the
    // speed to a maximum value
    double motor_rpm = shooter_wheel_rpm / RC::kShooterMotorGearRatio;
    const double MOTOR_MAX_RPM = 4500;
    if (motor_rpm >  MOTOR_MAX_RPM) motor_rpm = MOTOR_MAX_RPM;
    if (motor_rpm < 0) motor_rpm = 0;

    // Convert the speed to native units and pass to the speed controller
    double motor_speed_native = KoalafiedUtilities::TalonFXVelocityRpmToNative(motor_rpm);
    m_shooter_master_speed_controller->Set(ControlMode::Velocity, motor_speed_native);

    // Display the closed loop error
    double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
    frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);

    // Calculate and return whether we are at the correct speed
    const double kShootErrorPercentage = 2.0;
    double shooter_error_percent = 100.0*fabs((GetShooterRPM() - shooter_wheel_rpm)/shooter_wheel_rpm);
    return shooter_error_percent < kShootErrorPercentage;
}

int Shooter::GetShooterRPM() {
    // Get the motor speed in native units and convert it to shooter wheel rpm
    double motor_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double motor_speed_rpm = KoalafiedUtilities::TalonFXVelocityNativeToRpm(motor_speed_native);
    double shooter_wheel_rpm = motor_speed_rpm * RC::kShooterMotorGearRatio;
    return shooter_wheel_rpm;
}

void Shooter::TestDriveShooter(frc::Joystick* joystick) {

    const double MAX_SHOOTER_RPM = 6500.0; // TODO need to work this out properly
    const double MAX_MOTOR_RPM = MAX_SHOOTER_RPM / RC::kShooterMotorGearRatio; // TODO need to work this out properly

    // If the user holds the A button drive to the 
    bool run_at_speed = joystick->GetRawButton(RobotConfiguration::kJoystickAButton);
    
    if (run_at_speed) {
        double target_rpm = frc::SmartDashboard::GetNumber("Shooter RPM", 5000);            
        if (target_rpm < -MAX_SHOOTER_RPM) target_rpm = -MAX_SHOOTER_RPM;
        if (target_rpm > MAX_SHOOTER_RPM) target_rpm = MAX_SHOOTER_RPM;

        target_rpm /= RC::kShooterMotorGearRatio;

        // Use the tune drive to run the motor with the target RPM as it does the require RPM to native
        // calculations for us and logs helpful information to the dashboard
        KoalafiedUtilities::TuneDriveTalonFX(m_shooter_master_speed_controller, "Shooter", 1.0, target_rpm, true);
    }
    else {
        // Use the left joystick X axis to control the speed of the shooter. Do closed loop if the
        // left trigger button is held down.

        // COMMENTED OUT TO ALLOW THE HOOD TO USE left joystick X axis
        double stick = joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis);
        if (fabs(stick) < RC::kJoystickDeadzone) stick = 0.0;
        bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);
        KoalafiedUtilities::TuneDriveTalonFX(m_shooter_master_speed_controller, "Shooter", stick, MAX_MOTOR_RPM, close_loop);
    }
}
