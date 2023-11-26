//==============================================================================
// Arm.cpp
//==============================================================================

#include "Arm.h"


#include "../../util/KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>
#include <algorithm>




//==============================================================================
// Construction

Arm::Arm() {
	m_arm_speed_controller = NULL;
	m_arm_extension_set_inch = kArmExtensionNotSet;
}

Arm::~Arm() {

}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Arm::Setup() {
    std::cout << "Arm::Setup()\n";
    std::cout << "kArmInchPerEncoder " << kArmInchPerEncoder.value() << "\n";

	m_arm_speed_controller = new TalonFX(RC::kArmTalonId);

	// Set the arm current limits
	TalonFXConfiguration arm_configuration;
	arm_configuration.supplyCurrLimit = SupplyCurrentLimitConfiguration(
										true, 
										RC::kArmMotorContinuousCurrentLimit, 
										RC::kArmMotorPeakCurrentLimit, 
										RC::kArmMotorPeakCurrentDurationMs );

    // Set the nominal (aka minimum) close loop drive to 0.1.
    // This is tested as the minimum drive that will actually move the arm in the highest friction positions.
	// TODO Add contants for Arm nominal and peak values. Need to test vertically.
	// Arm moves with the smallest OL drive we can apply
	arm_configuration.nominalOutputForward = +0.2;
	arm_configuration.nominalOutputReverse = -0.2;
	arm_configuration.peakOutputForward = +1.0;
	arm_configuration.peakOutputReverse = -1.0;

    // Small closed loop ramp and larger for open loop (manual control)
	// TODO Consider slew rate limit
	arm_configuration.openloopRamp = RC::kArmMotorOpenLoopRampRateS;
	arm_configuration.closedloopRamp = RC::kArmMotorClosedLoopRampRateS;

	// PID parameters for Motion Magic control determined by velocity tuning
	arm_configuration.slot0.kF = 0.045;
	arm_configuration.slot0.kP = 0.01;
	arm_configuration.slot0.kI = 0.0;
	arm_configuration.slot0.kD = 0.0;

	// PID parameters for position control determined by set position tuning
    arm_configuration.slot1.kF = 0.0;
    arm_configuration.slot1.kP = 0.005;
    arm_configuration.slot1.kI = 0;
    arm_configuration.slot1.kD = 0;
    //arm_configuration.slot1.integralZone = 2_deg/kPivotDegreesPerEncoder;

	// Arm has limit switches in both directions and reset encode on the reverse one (vertical starting position)
	arm_configuration.forwardLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	arm_configuration.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	arm_configuration.reverseLimitSwitchSource = LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
	arm_configuration.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	arm_configuration.clearPositionOnLimitR = true;

	// Do configuration
	int error = m_arm_speed_controller->ConfigAllSettings(arm_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the pivot Falcon failed with code:  " << error << "\n";
    }

	// Brake mode to make it hold position when disabled
	m_arm_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // Initialise the encoder to zero, because the arm is at the bottom when the robot
    // starts up.
    m_arm_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

	// Set the close loop error
	const double kAllowableCloseLoopErrorInch = 0.2;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorInch / kArmInchPerEncoder.value();
	m_arm_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	SetupMotionMagic(1.0);
}

void Arm::Periodic()
{
	// Arm position and motor state
	// frc::SmartDashboard::PutNumber("Arm Extension", GetExtensionInch());
	// frc::SmartDashboard::PutNumber("Arm Current", m_arm_speed_controller->GetOutputCurrent());
	// frc::SmartDashboard::PutNumber("Arm Encoder", m_arm_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
	// frc::SmartDashboard::PutNumber("Arm Motor", m_arm_speed_controller->GetMotorOutputPercent());
	// // Faults arm_talon_faults;
	// Faults arm_talon_faults;
	// m_arm_speed_controller->GetFaults(arm_talon_faults);
	// // frc::SmartDashboard::PutBoolean("Arm RLimit", arm_talon_faults.ReverseLimitSwitch);
	// bool reverse_limit = m_arm_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
	// frc::SmartDashboard::PutBoolean("Arm RLimit", reverse_limit);
	// frc::SmartDashboard::PutBoolean("Arm FLimit", arm_talon_faults.ForwardSoftLimit);
}


//==============================================================================
// Operations

units::inch_t Arm::GetExtensionInch() const {
	// Get the extension in encoder units and convert to inches
	units::scalar_t extension_encoder = m_arm_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	units::inch_t extension_inch = extension_encoder * kArmInchPerEncoder;
	return extension_inch;
}

bool Arm::IsAtZeroLimitSwitch() const {
	return m_arm_speed_controller->IsRevLimitSwitchClosed();
}

bool Arm::IsAtFullLimitSwitch() const {
	return m_arm_speed_controller->IsFwdLimitSwitchClosed();
}

double Arm::GetArmDrive() {
	return m_arm_speed_controller->GetMotorOutputPercent();
}

double Arm::GetArmCurrent() const {
	return m_arm_speed_controller->GetStatorCurrent();
}

void Arm::SetExtensionInch(units::inch_t extension_inch, double velocity_factor) {
	if (extension_inch < 0.0_in) extension_inch = 0.0_in;
	if (extension_inch > RC::kArmMaximumExtensionInch) extension_inch = RC::kArmMaximumExtensionInch;

    // If we are going to 0 inches make sure we keep going until we hit the limit switch, so that the encoder
    // is reset and thus future movement is more accurate.
    // To achieve this we drive slowing (0.2) backwards if we are going to zero and are close (0.5_in)
    // to zero and have not yet hit the limit.
	units::inch_t current_extension = GetExtensionInch();
	if (current_extension < 0.5_in && extension_inch == 0_in && !m_arm_speed_controller->IsRevLimitSwitchClosed()) {
		m_arm_speed_controller->Set(ControlMode::PercentOutput, -0.2);
		return;
	}

    // Setup the velocity and acceleration for Motion Magic
    SetupMotionMagic(velocity_factor);

	// Record the arm extension
	m_arm_extension_set_inch = extension_inch;

	// Convert the extension to encoder units
	double arm_extension_native = m_arm_extension_set_inch / kArmInchPerEncoder;

	// Get the current angle and determine the current angle error
	units::inch_t extension_error = units::math::abs(extension_inch - GetExtensionInch());
	if (extension_error > 0.5_in) {
		// If the error is large drive to the position with Motion Magic so that the speed and
		// acceleration are limited.
		m_arm_speed_controller->SelectProfileSlot(0, RC::kTalonPidIdx);
		m_arm_speed_controller->Set(ControlMode::MotionMagic, arm_extension_native);
	} else {
		// If the error is small drive to the position with position close loop control and a different
		// set of PID parameters to minimise the error.
		m_arm_speed_controller->SelectProfileSlot(1, RC::kTalonPidIdx);
		m_arm_speed_controller->Set(ControlMode::Position, arm_extension_native);
	}

}

bool Arm::IsExtensionSet() {
	return (m_arm_extension_set_inch != kArmExtensionNotSet);
}

void Arm::ManualDriveArm(double percentage_output) {
	m_arm_extension_set_inch = kArmExtensionNotSet;
	m_arm_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Arm::TestDriveArm(frc::XboxController* joystick) {

     // Close loop test operation
	// - Up/Down - motion magic
	// - Left/Right - position
	double velocity_factor = 1.0;
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    SetExtensionInch( 0.0_in, velocity_factor); break;
		case RC::kJoystickPovDown:  SetExtensionInch(10.0_in, velocity_factor); break;
		case RC::kJoystickPovRight: SetExtensionInch( 5.0_in, velocity_factor); break;
		case RC::kJoystickPovLeft:  SetExtensionInch(15.0_in, velocity_factor); break;
		default: {
			bool closed_loop = joystick->GetLeftBumper();
			double arm_drive = 0.4 * frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone);
			double max_rpm = 5000;
			KoalafiedUtilities::TuneDriveTalonFX(m_arm_speed_controller, "Arm", arm_drive, max_rpm, closed_loop);
			break;
		}
	}
}

void Arm::TestDriveInch(ControlMode control_mode, units::inch_t extension_inch) {
	std::cout << "Test drive arm to " << extension_inch.value() << "\"\n";
	// Record the arm extension. This stops manual driving from interfering
	m_arm_extension_set_inch = extension_inch;

	m_arm_speed_controller->SelectProfileSlot(RC::kTalonTuneProfileSlotIdx, RC::kTalonPidIdx);

	// Calculate the extension in encode units and drive to it
	units::scalar_t arm_extension_native = m_arm_extension_set_inch / kArmInchPerEncoder;
	m_arm_speed_controller->Set(control_mode, arm_extension_native.value());
}


//==========================================================================
// Talon Setup

void  Arm::SetupMotionMagic(double velocity_factor) {
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
	const units::second_t kArmFullExtensionTimeS = 1.5_s;
	//const units::second_t kArmFullExtensionTimeS = 1.0_s; // Worked but is too fast until we fix the wrist to not flop.
	double velocity_inch_per_second = velocity_factor * RC::kArmMaximumExtensionInch.value()/kArmFullExtensionTimeS.value();
	double velocity_native = velocity_inch_per_second * RC::kTalonTimeBaseS/kArmInchPerEncoder.value();
	std::cout << "Arm motor cruise speed RPM " << KoalafiedUtilities::TalonFXVelocityNativeToRpm(velocity_native) << "\n";

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kArmAccelerationTimeS = 0.20;
	double acceleration_native = velocity_native/kArmAccelerationTimeS; 
 
	// Set the parameters in the Talon
	m_arm_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}


