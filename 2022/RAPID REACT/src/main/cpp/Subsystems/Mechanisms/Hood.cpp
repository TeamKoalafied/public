//==============================================================================
// Hood.cpp
//==============================================================================

#include "Hood.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;

//const double Hood::kHoodDegreesPerEncoder = 360.0 / (4096.0 * RC::kHoodRetractRatio);


//==============================================================================
// Construction

Hood::Hood()  {
    m_hood_speed_controller = NULL;
}

Hood::~Hood() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Hood::Setup() {
    std::cout << "Hood::Setup()\n";

    m_hood_speed_controller = new TalonSRX(RobotConfiguration::kHoodTalonId);

    TalonSRXConfiguration hood_configuration;

    hood_configuration.continuousCurrentLimit = 10;
    hood_configuration.peakCurrentLimit = 10;
    hood_configuration.peakCurrentDuration = 500;

    // Never run the hood motor above 50% as it starts to skip encoder counts
    hood_configuration.peakOutputForward = 0.5;
    hood_configuration.peakOutputReverse = -0.5;

    // Measured F is 12 to go up and 8 to go down
    hood_configuration.slot0.kF = 10.0;
    hood_configuration.slot0.kP = 10.0;
    // Feedback sensor 
    hood_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
 
    hood_configuration.clearPositionOnLimitR = true;

    // Do all configuration and log if it fails
    int error = m_hood_speed_controller->ConfigAllSettings(hood_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the hood Talon failed with code:  " << error << "\n";
    }
    
    // Perform non-configuration setup
    m_hood_speed_controller->SetSensorPhase(true); // Not reversed
    m_hood_speed_controller->EnableCurrentLimit(true);
	m_hood_speed_controller->SetNeutralMode(NeutralMode::Brake);
}

void Hood::Shutdown() {
    std::cout << "Hood::Shutdown()\n";
}

void Hood::Periodic() {
    double hood_motor_velocity_native = m_hood_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double hood_motor_velocity_rpm = hood_motor_velocity_native * 600 / 44.4;
    frc::SmartDashboard::PutNumber("New Motor Velocity (Native)", hood_motor_velocity_native);
    frc::SmartDashboard::PutNumber("New Motor Velocity (RPM)", hood_motor_velocity_rpm);    

    // Arm position and motor state
    frc::SmartDashboard::PutNumber("Hood Angle", GetHoodAngleDegrees());
    frc::SmartDashboard::PutNumber("Hood Current", m_hood_speed_controller->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Hood Encoder", m_hood_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
    frc::SmartDashboard::PutNumber("Hood Motor", m_hood_speed_controller->GetMotorOutputPercent());
    // Faults arm_talon_faults;
    Faults arm_talon_faults;
    m_hood_speed_controller->GetFaults(arm_talon_faults);
    // frc::SmartDashboard::PutBoolean("Arm RLimit", arm_talon_faults.ReverseLimitSwitch);
    bool reverse_limit = m_hood_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
    bool forward_limit = m_hood_speed_controller->GetSensorCollection().IsFwdLimitSwitchClosed();
    frc::SmartDashboard::PutBoolean("Hood RLimit", reverse_limit);
    frc::SmartDashboard::PutBoolean("Hood FLimit", forward_limit);

}

//==============================================================================
// Operations

const double BASE_ANGLE = 25.0;
const double pulses_per_revolution = 44.4 * 4.0;
const double gear_ratio = 380.0/15.0;
const double chain_ratio = 16.0/18.0;

const double PUSLES_PER_DEGREE = (pulses_per_revolution * gear_ratio * chain_ratio) /360.0;

double Hood::GetHoodAngleDegrees() {
    double sensor_position = m_hood_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);

    return BASE_ANGLE + sensor_position / PUSLES_PER_DEGREE;

    // 24.8degrees == 262counts

    

    //return 25.0 + 25.0 * sensor_position / 264.0;
}

bool Hood::SetHoodAngleDegrees(double angle_degrees) {
    double angle_encoder = (angle_degrees - BASE_ANGLE) * PUSLES_PER_DEGREE;
    frc::SmartDashboard::PutNumber("Hood Target Encoder", angle_encoder);
//    m_hood_speed_controller->Set(ControlMode::Position, angle_encoder);

    double current_angle = GetHoodAngleDegrees();
    double error = ::fabs(current_angle - angle_degrees);
    bool move_up = angle_degrees > current_angle;

    if (error < 1.0) {
        m_hood_speed_controller->Set(ControlMode::PercentOutput, 0);
        return true;
    } else if (error < 5.0) {
        const double SLOW_SPEED = 0.25;
        m_hood_speed_controller->Set(ControlMode::PercentOutput, move_up ? SLOW_SPEED : -SLOW_SPEED);
    } else {
        const double FAST_SPEED = 0.5;
        m_hood_speed_controller->Set(ControlMode::PercentOutput, move_up ? FAST_SPEED : -FAST_SPEED);
    }

    return false;
}

void Hood::OpenLoop(double speed) {
    m_hood_speed_controller->Set(ControlMode::PercentOutput, speed);
}

void Hood::TestDriveHood(frc::Joystick* joystick) {
    // double new_speed = joystick->GetRawAxis(RobotConfiguration::kJoystickRightXAxis);
    // m_hood_speed_controller->Set(ControlMode::PercentOutput, new_speed*0.2);

    double MAX_RPM = 420.0;

    // Use the right joystick X axis to control the speed of the hood. Do closed loop if the
    // left trigger button is held down.
    double stick = joystick->GetRawAxis(RobotConfiguration::kJoystickRightXAxis);
    if (fabs(stick) < RC::kJoystickDeadzone) stick = 0.0;
    stick *= 0.5;
    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);
    KoalafiedUtilities::TuneDriveTalon(m_hood_speed_controller, "Hood", stick, MAX_RPM, close_loop, 44.4 * 4.0);


    // Close loop test operation
	// - Up/Down - motion magic
	// - Left/Right - position
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    SetHoodAngleDegrees(25.0); break;
		case RC::kJoystickPovDown:  SetHoodAngleDegrees(35.0); break;
		case RC::kJoystickPovRight: SetHoodAngleDegrees(40.0); break;
		case RC::kJoystickPovLeft:  SetHoodAngleDegrees(50.0); break;
	}

}