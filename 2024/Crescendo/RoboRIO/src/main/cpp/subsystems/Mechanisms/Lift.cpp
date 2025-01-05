//==============================================================================
// Lift.cpp
//==============================================================================

#include "Lift.h"

#include <iostream>

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

#include <frc/XboxController.h>
#include <frc/MathUtil.h>


namespace RC = RobotConfiguration;

Lift::Lift() {
    m_lift_controller = nullptr;
}

Lift::~Lift() {

}

void Lift::Setup() {
    m_lift_controller = new TalonSRX(RC::kLiftTalonId);

    TalonSRXConfiguration lift_configuration;

    // Set the wrist current limits
	lift_configuration.peakCurrentLimit = RC::kLiftMotorPeakCurrentLimit;
	lift_configuration.peakCurrentDuration = RC::kLiftMotorPeakCurrentDurationMs;
	lift_configuration.continuousCurrentLimit = RC::kLiftMotorContinuousCurrentLimit;
    m_lift_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.4.
	lift_configuration.nominalOutputForward = +0.1;
	lift_configuration.nominalOutputReverse = -0.1;
	lift_configuration.peakOutputForward = +1.0;
	lift_configuration.peakOutputReverse = -1.0;

    // Set the speed controller ramp rate
	lift_configuration.openloopRamp = RC::kLiftMotorOpenLoopRampRateS;
	lift_configuration.closedloopRamp = RC::kLiftMotorClosedLoopRampRateS;

	// PID parameters for Motion Magic control 
	lift_configuration.slot0.kF = 0.6;
	lift_configuration.slot0.kP = 1.5;
	lift_configuration.slot0.kI = 0.0;
	lift_configuration.slot0.kD = 0.0;

    // PID parameters for Position control 
	lift_configuration.slot1.kF = 0.0;
	lift_configuration.slot1.kP = 0.005;
	lift_configuration.slot1.kI = 0.0;
	lift_configuration.slot1.kD = 0.0;

	lift_configuration.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
    lift_configuration.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
	lift_configuration.clearPositionOnLimitR = true;

	// Do configuration
	int error = m_lift_controller->ConfigAllSettings(lift_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the diverter Talon failed with code:  " << error << "\n";
    }

    // The wrist should brake in neutral, otherwise it will fall slowly.
    m_lift_controller->SetNeutralMode(NeutralMode::Brake);
	m_lift_controller->SetSensorPhase(true);
	m_lift_controller->SetSelectedSensorPosition(0);
    SetupMotionMagic(1.0);
}

void Lift::Periodic() {

}


void Lift::Shutdown() {

}

units::inch_t Lift::GetExtensionInch() const {
	// Get the extension in encoder units and convert to inches
	double extension_encoder = m_lift_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	units::inch_t extension_inch = units::inch_t(extension_encoder * kLiftInchPerEncoder);
	return extension_inch;
}

units::inch_t Lift::GetSetExtension() const {
	return m_lift_extension_set_inch;
}

bool Lift::IsAtZeroLimitSwitch() const {
	return m_lift_controller->IsRevLimitSwitchClosed();
}

bool Lift::IsAtFullLimitSwitch() const {
	return m_lift_controller->IsFwdLimitSwitchClosed();
}

bool Lift::IsExtensionSet() const {
    return (m_lift_extension_set_inch != kArmExtensionNotSet);
}

double Lift::GetOutput() const {
	return m_lift_controller->GetMotorOutputPercent();
}

double Lift::GetCurrent() const {
	return m_lift_controller->GetStatorCurrent();
}

void Lift::SetExtensionInch(units::inch_t extension_inch, double velocity_factor) {
	if (extension_inch < 0.0_in) extension_inch = 0.0_in;
	if (extension_inch > RC::kLiftMaximumExtensionInch) extension_inch = RC::kLiftMaximumExtensionInch;

    // If we are going to 0 inches make sure we keep going until we hit the limit switch, so that the encoder
    // is reset and thus future movement is more accurate.
    // To achieve this we drive slowing (0.2) backwards if we are going to zero and are close (0.5_in)
    // to zero and have not yet hit the limit.
	units::inch_t current_extension = GetExtensionInch();
	if (current_extension < 0.5_in && extension_inch == 0_in && !IsAtZeroLimitSwitch()) {
		m_lift_controller->Set(ControlMode::PercentOutput, 0.0);
		// std::cout << "Falling to limit switch\n";
		return;
	}

	if (extension_inch == 0_in && IsAtZeroLimitSwitch()) {
		m_lift_controller->Set(ControlMode::PercentOutput,0.0);
		// std::cout << "At zero limit\n";
		return;
	}

    // Setup the velocity and acceleration for Motion Magic
    SetupMotionMagic(velocity_factor);

	// Record the arm extension
	m_lift_extension_set_inch = extension_inch;

	// Convert the extension to encoder units
	double arm_extension_native = m_lift_extension_set_inch.value() / kLiftInchPerEncoder;

	// Drive the lift to the set position using motion magic
	m_lift_controller->SelectProfileSlot(0, RC::kTalonPidIdx);
	m_lift_controller->Set(ControlMode::MotionMagic, arm_extension_native);


}

void Lift::ManualDriveLift(double percentage_output) {
    m_lift_extension_set_inch = kArmExtensionNotSet;
    m_lift_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Lift::TestDriveLift(frc::XboxController* controller) {
	// Close loop test operation
	// - Up/Down - motion magic
	// - Left/Right - position
	switch (controller->GetPOV(0)) {
		case RC::kJoystickPovUp:    SetExtensionInch(3_in, 1.0); break;
		case RC::kJoystickPovDown:  SetExtensionInch(7_in, 1.0); break;
		case RC::kJoystickPovRight: SetExtensionInch(12_in, 1.0); break;
		case RC::kJoystickPovLeft: SetExtensionInch(16_in, 1.0); break;
   		default: {
            double MAX_RPM = 300.0;

            // Use the right joystick Y axis to control the speed of the Pivot. Do closed loop if the
            // left trigger button is held down.
			double pivot_drive = 0.5 * frc::ApplyDeadband(controller->GetRightY(), RC::kJoystickDeadzone);
            bool close_loop = controller->GetLeftBumper();
            KoalafiedUtilities::TuneDriveTalon(m_lift_controller, "Lift", pivot_drive, MAX_RPM, close_loop, 4096);
        }
	}
}

void Lift::SetupMotionMagic(double velocity_factor) {
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
	const units::second_t kLiftFullExtensionTimeS = 1.5_s;
	double velocity_inch_per_second = velocity_factor * RC::kLiftMaximumExtensionInch.value()/kLiftFullExtensionTimeS.value();
	double velocity_native = velocity_inch_per_second * RC::kTalonTimeBaseS/kLiftInchPerEncoder;
	std::cout << "Arm motor cruise speed RPM " << KoalafiedUtilities::TalonFXVelocityNativeToRpm(velocity_native) << "\n";

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kArmAccelerationTimeS = 0.20;
	double acceleration_native = velocity_native/kArmAccelerationTimeS; 
 
	// Set the parameters in the Talon
	m_lift_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_lift_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}