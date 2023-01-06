//==============================================================================
// Turret.cpp
//==============================================================================

#include "Turret.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;



//==============================================================================
// Construction

Turret::Turret()  {
    m_turret_speed_controller = NULL;
}

Turret::~Turret() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Turret::Setup() {
    std::cout << "Turret::Setup()\n";

    m_turret_speed_controller = new TalonSRX(RC::kTurretTalonId);

    TalonSRXConfiguration turret_configuration;

    turret_configuration.continuousCurrentLimit = 10;
    turret_configuration.peakCurrentLimit = 10;
    turret_configuration.peakCurrentDuration = 500;

    turret_configuration.slot0.kF = 0.12;
    turret_configuration.slot0.kP = 0.25;
    // Feedback sensor 
    turret_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;
 
    turret_configuration.clearPositionOnLimitR = true;

    // Do all configuration and log if it fails
    int error = m_turret_speed_controller->ConfigAllSettings(turret_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the turret Talon failed with code:  " << error << "\n";
    }
    
    // Perform non-configuration setup
    m_turret_speed_controller->SetSensorPhase(true); // Not reversed
    m_turret_speed_controller->EnableCurrentLimit(true);
	m_turret_speed_controller->SetNeutralMode(NeutralMode::Brake);
}

void Turret::Shutdown() {
    std::cout << "Turret::Shutdown()\n";
}

void Turret::Periodic() {
    // double turret_motor_velocity_native = m_turret_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    // double turret_motor_velocity_rpm = turret_motor_velocity_native * 600 / 44.4;
    // frc::SmartDashboard::PutNumber("New Motor Velocity (Native)", turret_motor_velocity_native);
    // frc::SmartDashboard::PutNumber("New Motor Velocity (RPM)", turret_motor_velocity_rpm);    

    // Arm position and motor state
    frc::SmartDashboard::PutNumber("Turret Angle", GetTurretAngleDegrees());
    frc::SmartDashboard::PutNumber("Turret Current", m_turret_speed_controller->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Turret Encoder", m_turret_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
    frc::SmartDashboard::PutNumber("Turret Motor", m_turret_speed_controller->GetMotorOutputPercent());

    bool reverse_limit = m_turret_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
    bool forward_limit = m_turret_speed_controller->GetSensorCollection().IsFwdLimitSwitchClosed();
    frc::SmartDashboard::PutBoolean("Turret RLimit", reverse_limit);
    frc::SmartDashboard::PutBoolean("Turret FLimit", forward_limit);
}

//==============================================================================
// Operations

const double kReverseLimitAngle = -175.0;
const double kForwardLimitAngle = 175.0;

double Turret::GetTurretAngleDegrees() {
    double sensor_position = m_turret_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);

    const double kDegreesPerEncoder = 360.0 / RC::kCtreEnocderCounts;
    const double kGearRatio = 234.0/25.0;


    return kReverseLimitAngle - sensor_position * kDegreesPerEncoder / kGearRatio;
}

bool Turret::SetTurretAngleDegrees(double angle_degrees, double max_speed) {
    // Clip the set angle to the allowed range. This ensures we are always going slow near the limit switches.
    if (angle_degrees < kReverseLimitAngle) angle_degrees = kReverseLimitAngle;
    if (angle_degrees > kForwardLimitAngle) angle_degrees = kForwardLimitAngle;

    // Calculate the 'error' distance to go to reach the desired angle and whether we are going up or down
    double current_angle = GetTurretAngleDegrees();
    double error = ::fabs(current_angle - angle_degrees);
    bool move_up = angle_degrees > current_angle;

    // Move the turret toward the required angle with a speed that depends on how far there is to go
    if (error < 1.0) {
        // If the error is less than 1 degree we are close enough so stop the turret and return that we are at the position
        m_turret_speed_controller->Set(ControlMode::PercentOutput, 0);
        return true;
    } else if (error < 2.0) {
        // If the error is less than 2 degress move at only a quarter of the maximum speed
        m_turret_speed_controller->Set(ControlMode::PercentOutput, move_up ? max_speed/4 : -max_speed/4);
    } else if (error < 5.0) {
        // If the error is less than 2 degress move at only a hlaf of the maximum speed
        m_turret_speed_controller->Set(ControlMode::PercentOutput, move_up ? max_speed/2 : -max_speed/2);
    } else {
        // If the error is greater than 5 degrees move at maximum speed
        m_turret_speed_controller->Set(ControlMode::PercentOutput, move_up ? max_speed : -max_speed);
    }

    return false;
}

void Turret::OpenLoop(double speed) {
    // If the turret is close the the ends limit the speed to a low value so it
    // doesn't jump past the limits switches. Make the angle tolerance fairly large
    // because maybe the encoder can be out of sync with the true angle.
    double current_angle = GetTurretAngleDegrees();
    const double kLimitTolerance = 5;
    const double kLimitSpeed = 0.2;
    if (current_angle < kReverseLimitAngle + kLimitTolerance || current_angle > kForwardLimitAngle - kLimitTolerance) {
        if (speed > kLimitSpeed) speed = kLimitSpeed;
        if (speed < -kLimitSpeed) speed = -kLimitSpeed;
    }

    m_turret_speed_controller->Set(ControlMode::PercentOutput, speed);
}

void Turret::TestDriveTurret(frc::Joystick* joystick) {
    // double new_speed = joystick->GetRawAxis(RobotConfiguration::kJoystickRightXAxis);
    // m_turret_speed_controller->Set(ControlMode::PercentOutput, new_speed*0.2);


    // Use the left joystick X axis to control the speed of the turret. Do closed loop if the
    // right trigger button is held down.
    const double MAX_RPM = 180.0; // TODO need to work this out properly
 
    double stick = joystick->GetRawAxis(RobotConfiguration::kJoystickLeftXAxis);
    if (fabs(stick) < RC::kJoystickDeadzone) stick = 0.0;
    stick *= 0.2;
    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickRTrigButton);
    KoalafiedUtilities::TuneDriveTalonSRX(m_turret_speed_controller, "Turret", stick, MAX_RPM, close_loop);


    // Close loop test operation
	// - Up/Down - motion magic
	// - Left/Right - position
	double velocity_factor = 0.2;
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    SetTurretAngleDegrees( 0.0, velocity_factor); break;
		case RC::kJoystickPovDown:  SetTurretAngleDegrees(160.0, velocity_factor); break;
		case RC::kJoystickPovRight: SetTurretAngleDegrees(90.0, velocity_factor); break;
		case RC::kJoystickPovLeft:  SetTurretAngleDegrees(-90.0, velocity_factor); break;
	}
}


//==========================================================================
// Talon Setup

void  Turret::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// Base the velocity on a time to travel the full rotation. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const double kTurretFullRotationTimeS = 2.0;
	double velocity_degrees_per_second = velocity_factor * 360.0/kTurretFullRotationTimeS;
    const double kDegreesPerEncoder = 360.0 / RC::kCtreEnocderCounts;
	double velocity_native = velocity_degrees_per_second * RC::kTalonTimeBaseS/kDegreesPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kArmAccelerationTimeS = 1.0;
	double acceleration_native = velocity_native/kArmAccelerationTimeS; 

	// Set the parameters in the Talon
	m_turret_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_turret_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}
