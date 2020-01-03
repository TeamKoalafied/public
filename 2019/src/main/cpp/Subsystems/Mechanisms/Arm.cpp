//==============================================================================
// Arm.cpp
//==============================================================================

#include "Arm.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>
#include <algorithm>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Arm::Arm() {
	m_arm_speed_controller = NULL;
	m_arm_extension_set_inch = -1;
}

Arm::~Arm() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Arm::Setup() {
    std::cout << "Arm::Setup()\n";

	m_arm_speed_controller = new TalonSRX(RC::kArmTalonId);

	// Set the arm current limits
	m_arm_speed_controller->ConfigPeakCurrentLimit(RC::kArmMotorPeakCurrentLimit, RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigPeakCurrentDuration(RC::kArmMotorPeakCurrentDurationMs, RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigContinuousCurrentLimit(RC::kArmMotorContinuousCurrentLimit, RC::kTalonTimeoutMs);
	m_arm_speed_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive to 0.2.
    // This is tested as the minimum drive that will actually move the arm in the highest friction positions.
	// TODO Add contants for Arm nominal and peak values. Need to test vertically.
    m_arm_speed_controller->ConfigNominalOutputForward(+0.1f, RC::kTalonTimeoutMs);
    m_arm_speed_controller->ConfigNominalOutputReverse(-0.1f, RC::kTalonTimeoutMs);
    m_arm_speed_controller->ConfigPeakOutputForward(+1.0f, RC::kTalonTimeoutMs);
    m_arm_speed_controller->ConfigPeakOutputReverse(-1.0f, RC::kTalonTimeoutMs);

    // Set the speed controller ramp.
    m_arm_speed_controller->ConfigOpenloopRamp(RC::kArmMotorOpenLoopRampRateS, RC::kTalonTimeoutMs);
    m_arm_speed_controller->ConfigClosedloopRamp(RC::kArmMotorClosedLoopRampRateS, RC::kTalonTimeoutMs);

	// The arm should brake in neutral. TODO Check this
	m_arm_speed_controller->SetNeutralMode(NeutralMode::Brake);

	// The arm uses a CTRE magnetic encoder, which operates in the same direction
	// to the motor drive.
	m_arm_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_arm_speed_controller->SetSensorPhase(false); // Not reversed

    // Initialise the encoder to zero, because the arm is at the bottom when the robot
    // starts up.
    m_arm_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

	// The arm has normally open limit switches the reverse direction only. Set
	// so that the encoder is zeroed any time the reverse limit switch is hit.
	// Set a forward direction soft limit at the maximum extension of the arm
	m_arm_speed_controller->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
														   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 
														   RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigClearPositionOnLimitR(true, RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigForwardSoftLimitEnable(true, RC::kTalonTimeoutMs);
	int arm_maximum_extension_encoder = RC::kArmMaximumExtensionInch / kArmInchPerEncoder;
	m_arm_speed_controller->ConfigForwardSoftLimitThreshold(arm_maximum_extension_encoder, RC::kTalonTimeoutMs);

	// Setup the close loop PIDF parameters
	const double kF = 0.4; // From experimentation
//	const double kP = 0.25;
//	const double kP = 0.068; // 50% motor output at 10" error
	const double kP = 0.04; // From experimentation
	const double kI = 0.001;// From experimentation
	const double kD = 0.0;
	m_arm_speed_controller->Config_kF(RC::kTalonRunProfileSlotIdx, kF, RC::kTalonTimeoutMs);
	m_arm_speed_controller->Config_kP(RC::kTalonRunProfileSlotIdx, kP, RC::kTalonTimeoutMs);
	m_arm_speed_controller->Config_kI(RC::kTalonRunProfileSlotIdx, kI, RC::kTalonTimeoutMs);
	m_arm_speed_controller->Config_kD(RC::kTalonRunProfileSlotIdx, kD, RC::kTalonTimeoutMs);
	m_arm_speed_controller->SelectProfileSlot(RC::kTalonRunProfileSlotIdx, RC::kTalonPidIdx);

	// Set the close loop error
	const double kAllowableCloseLoopErrorInch = 0.2;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorInch / kArmInchPerEncoder;
	m_arm_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	SetupMotionMagic(1.0);
}

void Arm::Shutdown() {
    std::cout << "Arm::Shutdown()\n";
	delete m_arm_speed_controller;
	m_arm_speed_controller = NULL;
}

void Arm::Periodic(bool show_dashboard)
{
	if (show_dashboard) {
		// Arm position and motor state
		frc::SmartDashboard::PutNumber("Arm Extension", GetExtensionInch());
		frc::SmartDashboard::PutNumber("Arm Current", m_arm_speed_controller->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("Arm Encoder", m_arm_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
		frc::SmartDashboard::PutNumber("Arm Motor", m_arm_speed_controller->GetMotorOutputPercent());
		// Faults arm_talon_faults;
		Faults arm_talon_faults;
		m_arm_speed_controller->GetFaults(arm_talon_faults);
		// frc::SmartDashboard::PutBoolean("Arm RLimit", arm_talon_faults.ReverseLimitSwitch);
		bool reverse_limit = m_arm_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
		frc::SmartDashboard::PutBoolean("Arm RLimit", reverse_limit);
		frc::SmartDashboard::PutBoolean("Arm FLimit", arm_talon_faults.ForwardSoftLimit);
	}
}


//==============================================================================
// Operations

double Arm::GetExtensionInch() {
	// Get the extension in encoder units and convert to inches
	int extension_encoder = m_arm_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	double extension_inch = extension_encoder * kArmInchPerEncoder;
	return extension_inch;
}

void Arm::SetExtensionInch(double extension_inch, double velocity_factor) {
	if (extension_inch < 0.0) extension_inch = 0.0;
	if (extension_inch > RC::kArmMaximumExtensionInch) extension_inch = RC::kArmMaximumExtensionInch;

	// std::cout << "Setting arm extension to " << extension_inch << "\"  velocity_factor " << velocity_factor << "\n";

	m_arm_speed_controller->SelectProfileSlot(RC::kTalonRunProfileSlotIdx, RC::kTalonPidIdx);

    // Setup the velocity and acceleration for Motion Magic
 	// Only doing full speed now so do not set up again
    //SetupMotionMagic(velocity_factor);

	// Record the arm extension
	m_arm_extension_set_inch = extension_inch;

	// Convert the extension to encoder units and drive to the position with Motion Magic
	double arm_extension_native = m_arm_extension_set_inch / kArmInchPerEncoder;
	m_arm_speed_controller->Set(ControlMode::MotionMagic, arm_extension_native);
}

bool Arm::IsExtensionSet() {
	return (m_arm_extension_set_inch != -1);
}

void Arm::ManualDriveArm(double percentage_output) {
	m_arm_extension_set_inch = -1;
	// Calculate the distance to the nearest end of the arm
	double extension_inch = GetExtensionInch();
	double end_distance_inch = std::min(extension_inch, RC::kArmMaximumExtensionInch - extension_inch);

	// If close to the end linearly reduce the speed
	const double END_DISTANCE_INCH = 5.0;
	const double END_SPEED = 0.2;
	if (end_distance_inch < END_DISTANCE_INCH) {
		double speed_factor = (end_distance_inch*1.0 + (END_DISTANCE_INCH - end_distance_inch)*END_SPEED)/END_DISTANCE_INCH;
		percentage_output *= speed_factor;
		// std::cout << "Arm::ManualDriveArm() speed_factor " << speed_factor << "\n";
	}

	// Drive the Talon at the calculated output
//	m_arm_speed_controller->Set(ControlMode::PercentOutput, percentage_output);

	// Convert the output to a velociy, relative to a maximum velocity and drive to that
	// This make the arm more controllable as it automatically adjusts for the large
	// changes in friction.
	const double kMaxArmSpeedInchPerS = 20.0;
	double velocity_inch_per_second = percentage_output * kMaxArmSpeedInchPerS;
	double velocity_native = velocity_inch_per_second * RC::kTalonTimeBaseS/kArmInchPerEncoder;
	m_arm_speed_controller->Set(ControlMode::Velocity, velocity_native);
}

void Arm::TestDriveArm(frc::Joystick* joystick) {
	// Drive arm using the triggers

    double left_trigger = joystick->GetRawAxis(RC::kJoystickLeftTriggerAxis);
    double right_trigger = joystick->GetRawAxis(RC::kJoystickRightTriggerAxis);

	if (left_trigger > 0.0 && right_trigger == 0.0) {
		m_arm_speed_controller->Set(ControlMode::PercentOutput, -left_trigger);
		m_arm_extension_set_inch = -1;
		// std::cout << "Manual drive " << -left_trigger << "\n";
	} else if (right_trigger > 0.0 && left_trigger == 0.0) {
		m_arm_speed_controller->Set(ControlMode::PercentOutput, right_trigger);
		m_arm_extension_set_inch = -1;
		// std::cout << "Manual drive " << right_trigger << "\n";
	} else {
		if (m_arm_extension_set_inch == -1) {
			//m_arm_speed_controller->Set(ControlMode::PercentOutput, 0.0);
			//std::cout << "Manual drive to zero\n";

			// Lock the arm at the current position by doing close loop drive to it.
			int current_extension = GetExtensionInch();
			SetExtensionInch(current_extension, 1.0);
		}
	}

	static int counter = 0;
	if (counter++ % 10 == 0 && fabs(m_arm_speed_controller->GetMotorOutputVoltage()) > 0.05) {
		KoalafiedUtilities::CalculateAndLogF(m_arm_speed_controller, kArmInchPerEncoder/RC::kTalonTimeBaseS, "Arm");
	}

    // Close loop test operation
	// - Up/Down - motion magic
	// - Left/Right - position
	double velocity_factor = 1.0;
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    SetExtensionInch( 0.0, velocity_factor); break;
		case RC::kJoystickPovDown:  SetExtensionInch(20.0, velocity_factor); break;
		case RC::kJoystickPovRight: SetExtensionInch(10.0, velocity_factor); break;
		case RC::kJoystickPovLeft:  SetExtensionInch(30.0, velocity_factor); break;
		/// case RC::kJoystickPovUp:    Arm::TestDriveInch(ControlMode::MotionMagic, 20.0); break;
		// case RC::kJoystickPovDown:  Arm::TestDriveInch(ControlMode::MotionMagic, 10.0); break;
		// case RC::kJoystickPovRight: Arm::TestDriveInch(ControlMode::Position, 20.0); break;
		// case RC::kJoystickPovLeft:  Arm::TestDriveInch(ControlMode::Position, 10.0); break;
	}
}

void Arm::TestDriveInch(ControlMode control_mode, double extension_inch) {
	std::cout << "Test drive arm to " << extension_inch << "\"\n";
	// Record the arm extension. This stops manual driving from interfering
	m_arm_extension_set_inch = extension_inch;

	m_arm_speed_controller->SelectProfileSlot(RC::kTalonTuneProfileSlotIdx, RC::kTalonPidIdx);

	// Calculate the extension in encode units and drive to it
	double arm_extension_native = m_arm_extension_set_inch / kArmInchPerEncoder;
	m_arm_speed_controller->Set(control_mode, arm_extension_native);
}


//==========================================================================
// Talon Setup

void  Arm::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// Base the velocity on a time to travel the full extension distance. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const double kArmFullExtensionTimeS = 3.0;
	double velocity_inch_per_second = velocity_factor * RC::kArmMaximumExtensionInch/kArmFullExtensionTimeS;
	double velocity_native = velocity_inch_per_second * RC::kTalonTimeBaseS/kArmInchPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kArmAccelerationTimeS = 1.0;
	double acceleration_native = velocity_native/kArmAccelerationTimeS; 

	// Set the parameters in the Talon
	m_arm_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_arm_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}


