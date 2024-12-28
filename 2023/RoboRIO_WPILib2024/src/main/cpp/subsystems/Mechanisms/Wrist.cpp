//==============================================================================
// Wrist.cpp
//==============================================================================

#include "Wrist.h"

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "../../Phoenix5Header.h"
#include <fstream>
#include <frc/MathUtil.h>
#include <iostream>
#include <iomanip>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Wrist::Wrist()  {
    m_wrist_speed_controller = NULL;
    m_wrist_rotation_set_angle = kWristAngleNotSet; // a nonsensical value to indicate we are not trying to drive
}

Wrist::~Wrist() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Wrist::Setup() {
    std::cout << "Wrist::Setup()\n";

    m_wrist_speed_controller = new TalonSRX(RC::kWristTalonId);

	TalonSRXConfiguration wrist_configuration;

    // Set the wrist current limits
	wrist_configuration.peakCurrentLimit = RC::kWristMotorPeakCurrentLimit;
	wrist_configuration.peakCurrentDuration = RC::kWristMotorPeakCurrentDurationMs;
	wrist_configuration.continuousCurrentLimit = RC::kWristMotorContinuousCurrentLimit;
    m_wrist_speed_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.4.
	wrist_configuration.nominalOutputForward = +0.1;
	wrist_configuration.nominalOutputReverse = -0.1;
	wrist_configuration.peakOutputForward = +1.0;
	wrist_configuration.peakOutputReverse = -1.0;

    // Set the speed controller ramp rate
	wrist_configuration.openloopRamp = RC::kWristMotorRampRateDriverS;
	wrist_configuration.closedloopRamp = 0.05;

	// PID parameters for Motion Magic control determined by velocity tuning
	wrist_configuration.slot0.kF = 15.0;
	wrist_configuration.slot0.kP = 20.0;
	wrist_configuration.slot0.kI = 0.0;
	wrist_configuration.slot0.kD = 0.0;

	// PID parameters for position control determined by set position tuning
    wrist_configuration.slot1.kF = 0.0;
    wrist_configuration.slot1.kP = 1;
    wrist_configuration.slot1.kI = 0;
    wrist_configuration.slot1.kD = 0;
    wrist_configuration.slot1.integralZone = 2_deg/kWristDegreesPerEncoder;

	// The wrist has one limit switch in the forward direction that resets the encoder.
	// In the reverse direction we use a soft limit at 105 degrees.
	wrist_configuration.forwardLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	wrist_configuration.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	wrist_configuration.clearPositionOnLimitF = true;
	wrist_configuration.reverseSoftLimitEnable = true;
	int wrist_minimum_angle_encoder = ConvertAngleDegreesToEncoder(kWristMaximumReverseDegrees);
	wrist_configuration.reverseSoftLimitThreshold = wrist_minimum_angle_encoder;

	// JE motors use a quadrature encoder
    wrist_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;

	// Do configuration
	int error = m_wrist_speed_controller->ConfigAllSettings(wrist_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the pivot Falcon failed with code:  " << error << "\n";
    }

    // The wrist should brake in neutral, otherwise it will fall slowly.
    m_wrist_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // The JE motor encoder is wired so that the sensor is reversed (A to 1, B to 2)
    m_wrist_speed_controller->SetSensorPhase(true);

	// Initialise the encoder to zero as start position is horizontal (against the limit switch)
    m_wrist_speed_controller->SetSelectedSensorPosition(0);

	// Set the close loop error. NOTE: This is calculating a delta angle, not an absolute one
	// and hence does not use ConvertAngleDegreesToEncoder().
	const units::degree_t kAllowableCloseLoopErrorDegrees = 1_deg;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorDegrees / kWristDegreesPerEncoder;
	m_wrist_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	m_motion_magic_velocity_factor = -1; // Clear so that it is definitely set
	SetupMotionMagic(1.0);
}

void Wrist::Shutdown() {
    std::cout << "Wrist::Shutdown()\n";
}

void Wrist::Periodic()
{
	// Log any faults on the talon
	Faults wrist_talon_faults;
	m_wrist_speed_controller->GetFaults(wrist_talon_faults);
	if (wrist_talon_faults.HasAnyFault()) {
		//std::cout << "Wrist Faults " << wrist_talon_faults.ToString() << "\n";
	}

	frc::SmartDashboard::PutNumber("WristEncoder", m_wrist_speed_controller->GetSelectedSensorPosition());
	frc::SmartDashboard::PutNumber("WristVelocity", m_wrist_speed_controller->GetSelectedSensorVelocity());
}


//==============================================================================
// Operations

units::degree_t Wrist::GetWristAngleDegrees() const {
	return ConvertEncoderToAngleDegrees(m_wrist_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
}

bool Wrist::IsAtZeroLimitSwitch() const {
	return m_wrist_speed_controller->IsFwdLimitSwitchClosed();
}

bool Wrist::IsAtFullLimitSwitch() const {
	return m_wrist_speed_controller->IsRevLimitSwitchClosed();
}

void Wrist::DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor)
{
	if (angle_degrees < kWristMaximumReverseDegrees) angle_degrees = kWristMaximumReverseDegrees;

	// Do not limit the forward direction of the wrist. This is safe because we have a limit switch in
	// this direction and this allows us to manually drive back to it if the robot is started in the
	// wrong configuration.
	//if (angle_degrees > kWristMaximumForwardDegrees) angle_degrees = kWristMaximumForwardDegrees;
    //std::cout << "Setting wrist angle to " << angle_degrees << "  velocity_factor " << velocity_factor << "\n";

    // If we are going to 0 degrees make sure we keep going until we hit the limit switch, so that the encoder
    // is reset and thus future movement is more accurate.
    // To achieve this we drive slowing (0.1) forward (wrist down) if we are going to zero and are close (0.5_deg)
    // to zero and have not yet hit the limit.
	units::degree_t current_angle = GetWristAngleDegrees();
	// if (current_angle > -0.5_deg && angle_degrees == 0_deg && !m_wrist_speed_controller->IsFwdLimitSwitchClosed()) {
	// 	m_wrist_speed_controller->Set(ControlMode::PercentOutput, 0.1);
	// 	return;
	// }

    // Setup the velocity and acceleration for Motion Magic
    SetupMotionMagic(velocity_factor);

	// Record the wrist angle
	m_wrist_rotation_set_angle = angle_degrees;

	// Convert the angle to encoder units and drive to the position with Motion Magic
    double wrist_angle_native = ConvertAngleDegreesToEncoder(m_wrist_rotation_set_angle); 

	// Get the current angle and determine the current angle
	units::degree_t angle_error = units::math::abs(angle_degrees - current_angle);
	if (angle_error > 2_deg) {
		// If the error is large drive to the position with Motion Magic so that the speed and
		// acceleration are limited.
        m_wrist_speed_controller->SelectProfileSlot(0, RC::kTalonPidIdx);
    	m_wrist_speed_controller->Set(ControlMode::MotionMagic, wrist_angle_native);
    } else {
		// If the error is small drive to the position with position close loop control and a different
		// set of PID parameters to minimise the error.
        m_wrist_speed_controller->SelectProfileSlot(1, RC::kTalonPidIdx);
    	m_wrist_speed_controller->Set(ControlMode::Position, wrist_angle_native);
    }
}

bool Wrist::IsAngleSet() {
	return (m_wrist_rotation_set_angle != kWristAngleNotSet);
}


//==========================================================================
// Shuffleboard & Logging

double Wrist::GetWristDrive() {
	return m_wrist_speed_controller->GetMotorOutputPercent();
}

double Wrist::GetWristCurrent() const {
	return m_wrist_speed_controller->GetStatorCurrent();
}


//==========================================================================
// Testing

void Wrist::ManualDriveWrist(double percentage_speed) {
  	m_wrist_rotation_set_angle = kWristAngleNotSet;
	m_wrist_speed_controller->Set(ControlMode::PercentOutput, percentage_speed*0.15);
}

void Wrist::TestDriveWrist(frc::XboxController* joystick)
{
	double velocity_factor = 1.0;
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    DriveToAngleDegrees( 0.0_deg, velocity_factor); break;
		case RC::kJoystickPovDown:  DriveToAngleDegrees(-22.5_deg, velocity_factor); break;
		case RC::kJoystickPovRight: DriveToAngleDegrees(-45.0_deg, velocity_factor); break;
		case RC::kJoystickPovLeft:  DriveToAngleDegrees(-90.0_deg, velocity_factor); break;
		default: {
			bool closed_loop = joystick->GetLeftBumper();
			double pivot_drive = 0.5 * frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone);
			double max_rpm = 120;
			KoalafiedUtilities::TuneDriveTalon(m_wrist_speed_controller, "Winch", pivot_drive, max_rpm, closed_loop, 44.4 * 4.0);
			break;
		}
	}
}

void Wrist::TestDriveAngle(ControlMode control_mode, units::degree_t extension_angle) {
	std::cout << "Test drive wrist to " << extension_angle.value() << "\"\n";
	// Record the wrist angle. This stops manual driving from interfering.
	m_wrist_rotation_set_angle = extension_angle;

	m_wrist_speed_controller->SelectProfileSlot(RC::kTalonTuneProfileSlotIdx, RC::kTalonPidIdx);

	// Calculate the angle in encode units and drive to it
	double wrist_rotation_native = ConvertAngleDegreesToEncoder(m_wrist_rotation_set_angle);
	m_wrist_speed_controller->Set(control_mode, wrist_rotation_native); 

    std::cout << "wrist_rotation_native " << wrist_rotation_native <<  "\"\n";
    std::cout << "current_encoder_count " << m_wrist_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx) <<  "\"\n";
}


//==========================================================================
// Talon Setup

void  Wrist::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// If the factor is already set do nothing. This is critical as is does not change much, but
	// setting it every update period results in loop time overruns.
	if (m_motion_magic_velocity_factor == velocity_factor) return;
	m_motion_magic_velocity_factor = velocity_factor;

	// Base the velocity on a time to travel the full extension distance. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const units::second_t kWristFullRotationTimeS = 1.7_s;
	//const units::second_t kWristFullRotationTimeS = 1.4_s; // Worked but is too fast until we fix the wrist to not flop.
    units::degree_t wrist_angle_range = kWristMaximumForwardDegrees - kWristMaximumReverseDegrees;
	units::degrees_per_second_t velocity_degrees_per_second = velocity_factor * wrist_angle_range/kWristFullRotationTimeS;
	double velocity_native = velocity_degrees_per_second * units::second_t(RC::kTalonTimeBaseS) / kWristDegreesPerEncoder;
	//std::cout << "Wrist cruise velocity native " << velocity_native << "\n";

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kWristAccelerationTimeS = 0.25;
	double acceleration_native = velocity_native/kWristAccelerationTimeS; 

	// Set the parameters in the Talon
	m_wrist_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

double Wrist::ConvertAngleDegreesToEncoder(units::degree_t angle_degrees) {
	return angle_degrees / kWristDegreesPerEncoder;
}

units::degree_t Wrist::ConvertEncoderToAngleDegrees(units::scalar_t encoder_position) const {
	return encoder_position * kWristDegreesPerEncoder;
}
