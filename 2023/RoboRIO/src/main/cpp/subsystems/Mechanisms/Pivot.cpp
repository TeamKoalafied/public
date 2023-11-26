//==============================================================================
// Pivot.cpp
//==============================================================================

#include "Pivot.h"

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"
        
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

namespace RC = RobotConfiguration;

const units::degree_t Pivot::kPivotDegreesPerEncoder = 360.0_deg / (RC::kTalonFXEnocderCounts * RC::kPivotGearRatio);


//==============================================================================
// Construction

Pivot::Pivot() {
    m_pivot_speed_controller = NULL;
    m_pivot_rotation_set_angle = kPivotAngleNotSet; // a nonsensical value to indicate we are not trying to drive

	// The limit switches are not triggered in the start position
	m_forward_limit = false;
	m_reverse_limit = false;
}

Pivot::~Pivot() {

}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Pivot::Setup() {
    std::cout << "Pivot::Setup()\n";

    // Create the motor controller and then setup the configuration
    m_pivot_speed_controller = new TalonFX(RC::kPivotTalonId);
	TalonFXConfiguration pivot_configuration;

    // Nominal output is small so that the pivot can position in fine increments. Peak
    // output is limited because the pivot is a big scary mechanism that does not need
    // to move a full speed.
    pivot_configuration.nominalOutputForward = +0.05;
    pivot_configuration.nominalOutputReverse = -0.05;
    pivot_configuration.peakOutputForward = + 0.5;
    pivot_configuration.peakOutputReverse = -0.5;

    // Small closed loop ramp and larger for open loop (manual control)
	// TODO Consider slew rate limit
    pivot_configuration.closedloopRamp = RC::kPivotMotorClosedLoopRampRateS;
    pivot_configuration.openloopRamp = RC::kPivotMotorOpenLoopRampRateS;

	// Current limits set well in excess of what seems to be required
	// Mostly current is below 5A
    pivot_configuration.supplyCurrLimit = SupplyCurrentLimitConfiguration(true, 
        RC::kPivotMotorContinuousCurrentLimit,
        RC::kPivotMotorPeakCurrentLimit,
        RC::kPivotMotorPeakCurrentDurationMs);
    
	// PID parameters for Motion Magic control determined by velocity tuning
    pivot_configuration.slot0.kF = 0.06;
    pivot_configuration.slot0.kP = 0.06;
    pivot_configuration.slot0.kI = 0;
    pivot_configuration.slot0.kD = 0;

	// PID parameters for position control determined by set position tuning
    pivot_configuration.slot1.kF = 0.0;
    pivot_configuration.slot1.kP = 0.01;
    pivot_configuration.slot1.kI = 0;
    pivot_configuration.slot1.kD = 0;
    pivot_configuration.slot1.integralZone = 2_deg/kPivotDegreesPerEncoder;

	// Pivot has limit switches in both directions and reset encode on the reverse one (vertical starting position)
	pivot_configuration.forwardLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	pivot_configuration.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	pivot_configuration.clearPositionOnLimitF = true;
	pivot_configuration.reverseLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	pivot_configuration.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;

	// Set a reverse soft limit for the pivot so that it does not drive into the tall cone picky spike
	pivot_configuration.reverseSoftLimitEnable = true;
	int pivot_minimum_angle_encoder = ConvertAngleDegreesToEncoder(RC::kPivotMaximumReverseDegrees);
	pivot_configuration.reverseSoftLimitThreshold = pivot_minimum_angle_encoder;

	// Do configuration
    int error = m_pivot_speed_controller->ConfigAllSettings(pivot_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the pivot Falcon failed with code:  " << error << "\n";
    }

	// Brake mode to make it holds position when disabled
    m_pivot_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // Initialise the encoder to zero start config is arm is pointing straight up.
    m_pivot_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

	// Set the close loop error. NOTE: This is calculating a delta angle, not an absolute one
	// and hence does not use ConvertAngleDegreesToEncoder().
	const units::degree_t kAllowableCloseLoopErrorDegrees = 1_deg;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorDegrees / kPivotDegreesPerEncoder;
	m_pivot_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	SetupMotionMagic(1.0);
}

void Pivot::Periodic() {

}

//==============================================================================
// Operations

units::degree_t Pivot::GetPivotAngleDegrees() const {
   	// Get the angle in encoder units and convert to degrees
	int angle_encoder = m_pivot_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	units::degree_t angle_degrees = ConvertEncoderToAngleDegrees(angle_encoder);
	return angle_degrees;
}

bool Pivot::IsAtZeroLimitSwitch() const {
	return m_pivot_speed_controller->IsFwdLimitSwitchClosed();
}

bool Pivot::IsAtFullLimitSwitch() const {
	return m_pivot_speed_controller->IsRevLimitSwitchClosed();
}

double Pivot::GetPivotDrive() {
	return m_pivot_speed_controller->GetMotorOutputPercent();
}

double Pivot::GetPivotCurrent() const {
	return m_pivot_speed_controller->GetStatorCurrent();
}

void Pivot::DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor) {
    // Limit the target angle to the allowed range. There are also limit switches but they
    // would produce a sudden stop.
	if (angle_degrees < RC::kPivotMaximumReverseDegrees) angle_degrees = RC::kPivotMaximumReverseDegrees;
	if (angle_degrees > RC::kPivotMaximumForwardDegrees) angle_degrees = RC::kPivotMaximumForwardDegrees;
    // std::cout << "Setting pivot angle to " << angle_degrees << "  velocity_factor " << velocity_factor << "\n";


	// If driving towards the low cone pickup position and we are close, just turn off brake mode
	// and coast the rest of the way. This means that we rest on the block that supports the arm
	// rather than driving into it. 
	units::degree_t current_angle = GetPivotAngleDegrees();
	if (current_angle < -86_deg && angle_degrees == -88_deg) {
		m_pivot_speed_controller->SetNeutralMode(NeutralMode::Coast);
		m_pivot_speed_controller->Set(ControlMode::PercentOutput, 0.0);
		return;
	}

	// Turn on brake mode if anything else is happening
	m_pivot_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // If we are going to 0 degrees make sure we keep going until we hit the limit switch, otherwise
    // the pivot tends to stop slightly short of vertical due to slop in the chain, which makes the
    // position when we move again less accurate.
    // To achieve this we drive slowing (0.1) upwards if we are going to zero and are close (0.5_deg)
    // to zero and have not yet hit the limit.
	if (current_angle > -0.5_deg && angle_degrees == 0_deg && !m_pivot_speed_controller->IsFwdLimitSwitchClosed()) {
		m_pivot_speed_controller->Set(ControlMode::PercentOutput, 0.1);
		return;
	}

    // Setup the velocity and acceleration for Motion Magic
	// Only doing full speed now so do not set up again
    SetupMotionMagic(velocity_factor);

	// Record the pivot rotation
	m_pivot_rotation_set_angle = angle_degrees;

	// Convert the angle to encoder units
	double pivot_angle_native = ConvertAngleDegreesToEncoder(m_pivot_rotation_set_angle); 

	// Get the current angle and determine the current angle error
	units::degree_t angle_error = units::math::abs(angle_degrees - current_angle);
	if (angle_error > 2_deg) {
		// If the error is large drive to the position with Motion Magic so that the speed and
		// acceleration are limited.
		m_pivot_speed_controller->SelectProfileSlot(0, RC::kTalonPidIdx);
		m_pivot_speed_controller->Set(ControlMode::MotionMagic, pivot_angle_native);

		//std::cout << "Pivot Drive to Angle - Motion Magic " << angle_degrees.value() << "\n";
	} else {
		// If the error is small drive to the position with position close loop control and a different
		// set of PID parameters to minimise the error.
		m_pivot_speed_controller->SelectProfileSlot(1, RC::kTalonPidIdx);
		m_pivot_speed_controller->Set(ControlMode::Position, pivot_angle_native);

		//std::cout << "Pivot Drive to Angle - Position " << angle_degrees.value() << "\n";
	}
}

bool Pivot::IsAngleSet() {
	return (m_pivot_rotation_set_angle != kPivotAngleNotSet);
}

void Pivot::ManualDrivePivot(double percentage_output) {
	if (m_pivot_rotation_set_angle != kPivotAngleNotSet) std::cout << "Pivot Manual Drive " << percentage_output << "\n";
	m_pivot_rotation_set_angle = kPivotAngleNotSet;
    m_pivot_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Pivot::TestDrivePivot(frc::XboxController* joystick) {
    // Drive the pivot using the right Y axis
    // double pivot_drive = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    // if (fabs(pivot_drive) < RC::kJoystickDeadzone) pivot_drive = 0.0;
    // pivot_drive = KoalafiedUtilities::PowerAdjust(pivot_drive, RobotConfiguration::kPivotJoystickPower);
	// if (pivot_drive != 0.0) {
	//     ManualDrivePivot(pivot_drive);
	// 	// std::cout << "Manual drive " << pivot_drive << "\n";
 	// } else {
    //     if (!IsAngleSet()) {
	//         //ManualDrivePivot(0.0);
    //         //std::cout << "Zeroing drive " << "\n";

	// 		// Lock the pivot at the current position by doing close loop drive to it.
	// 		double angle_degrees = GetPivotAngleDegrees();
	// 		DriveToAngleDegrees(angle_degrees, 0.8);
    //     }
	// }

	// Log the F value occasionally
	bool closed_loop = joystick->GetLeftBumper();
	double pivot_drive = 0.2 * frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone);
	double max_rpm = 120;
	KoalafiedUtilities::TuneDriveTalonFX(m_pivot_speed_controller, "Pivot", pivot_drive, max_rpm, closed_loop);

    // Close loop test operation
	// double velocity_factor = 0.8;
	// switch (joystick->GetPOV(0)) {
	// 	case RC::kJoystickPovUp:    DriveToAngleDegrees(15.0, velocity_factor); break;
	// 	case RC::kJoystickPovDown:  DriveToAngleDegrees(-15.0, velocity_factor); break;
	// 	case RC::kJoystickPovRight: DriveToAngleDegrees(45.0, velocity_factor); break;
	// 	case RC::kJoystickPovLeft:  DriveToAngleDegrees(-45.0, velocity_factor); break;
	// }
}


//==========================================================================
// Talon Setup

void  Pivot::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// If the factor is already set do nothing. This is critical as is does not change much, but
	// setting it every update period results in loop time overruns.
	if (m_motion_magic_velocity_factor == velocity_factor) return;
	m_motion_magic_velocity_factor = velocity_factor;

	// Base the velocity on a time to travel the full allowed angle. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const units::second_t kPivotFullRotationTimeS = 2.0_s; // was 3.0
	//const units::second_t kPivotFullRotationTimeS = 1.6_s; // Worked but is too fast until we fix the wrist to not flop.
    units::degree_t pivot_angle_range = RC::kPivotMaximumForwardDegrees - RC::kPivotMaximumReverseDegrees;

	units::degrees_per_second_t velocity_degrees_per_second = velocity_factor * pivot_angle_range/kPivotFullRotationTimeS;
	double velocity_native = velocity_degrees_per_second * units::second_t(RC::kTalonTimeBaseS) / kPivotDegreesPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kPivotAccelerationTimeS = 0.25; // was 1.0
	double acceleration_native = velocity_native/kPivotAccelerationTimeS; 

	// Set the parameters in the Talon
	m_pivot_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

double Pivot::ConvertAngleDegreesToEncoder(units::degree_t angle_degrees) {
	return angle_degrees / kPivotDegreesPerEncoder;
}

units::degree_t Pivot::ConvertEncoderToAngleDegrees(double encoder_position) const {
	return encoder_position * kPivotDegreesPerEncoder;
}
