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
    m_shooter_slave_speed_controller = new TalonFX(RobotConfiguration::kShooterSlaveTalonId);
    m_shooter_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kShooterMasterTalonId);
    m_shooter_slave_speed_controller->SetInverted(true);

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
    shooter_configuration.slot0.kF = 0.048;
    shooter_configuration.slot0.kP = 0.25;
    shooter_configuration.slot0.kI = 0.0;
    shooter_configuration.slot0.kD = 0.0;
    
    // Configure the two motors        
    int error = m_shooter_master_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the shooter Talon failed with code:  " << error << "\n";
    }
    int error2 = m_shooter_slave_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error2 != 0) {
        std::cout << "Configuration of the shooter slave Talon failed with code:  " << error2 << "\n";
    }

    // Set the motors to coast. Braking is unnecessary and will cause huge back EMF
	m_shooter_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
    m_shooter_slave_speed_controller->SetNeutralMode(NeutralMode::Coast);

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
    m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, -percentage_speed);
}

void Shooter::DriveShooterClosedLoop(double shooter_wheel_rpm) {
    // Convert the shooter shaft speed to a motor speed using the gear ratio and clip the
    // speed to a maximum value
    double motor_rpm = shooter_wheel_rpm / RC::kShooterMotorGearRatio;
    const double MOTOR_MAX_RPM = 4500;
    if (motor_rpm >  MOTOR_MAX_RPM) motor_rpm = MOTOR_MAX_RPM;
    if (motor_rpm < -MOTOR_MAX_RPM) motor_rpm = -MOTOR_MAX_RPM;

    // Convert the speed to native units and pass to the speed controller
    double motor_speed_native = KoalafiedUtilities::TalonFXVelocityRpmToNative(motor_rpm);
    m_shooter_master_speed_controller->Set(ControlMode::Velocity, -motor_speed_native);

    // Display the closed loop error
    double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
    frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);
}

int Shooter::GetShooterRPM() {
    // Get the motor speed in native units and convert it to shooter wheel rpm
    double motor_speed_native = -m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double motor_speed_rpm = KoalafiedUtilities::TalonFXVelocityNativeToRpm(motor_speed_native);
    double shooter_wheel_rpm = motor_speed_rpm * RC::kShooterMotorGearRatio;
    return shooter_wheel_rpm;
}

void Shooter::TestDriveShooter(frc::Joystick* joystick) {  
    double shooter_wheel_rpm = (frc::SmartDashboard::GetNumber("Shooter RPM", 6000.0));

    if (joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis) > 0.0) {
        // Run in close loop and report the error margin

        // Do closed loop velocity control and set a desired speed from
        // our movement value.
        DriveShooterClosedLoop(shooter_wheel_rpm);

        // Get the close loop error and convert to RPM
        double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
        frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);    
    } else {

        double shooter_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(shooter_drive) < RC::kJoystickDeadzone) shooter_drive = 0.0;

        m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, shooter_drive);
        
        // Run in open loo
    // if (joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis)){
    //     m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis));
    // } else {p and calculate a value for F

        // $5 for free - afghan32

        // Calculate the motor output voltage as a fraction
        double motor_output = m_shooter_master_speed_controller->GetMotorOutputVoltage()/
                                m_shooter_master_speed_controller->GetBusVoltage();

        // Get the speed in RPM and convert to the native units of encode counts
        // per 100ms time period (see TSSRM page 88).
        double speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
        // double speed_rpm = speed_native * 60.0 *10.0 / 2048.0;

        // Calculate a feed forward gain (F) for this speed
        double F;
        if (speed_native == 0) {
            F = 0;
        } else
        {
            F = motor_output * 1023.0/speed_native;
        }
        
        // output F to the networktables
        frc::SmartDashboard::PutNumber("Shooter F", F);
        // Output the values if required
        // sstd::cout << "shooter F: " << F << std::endl;
    // }
    }
}
