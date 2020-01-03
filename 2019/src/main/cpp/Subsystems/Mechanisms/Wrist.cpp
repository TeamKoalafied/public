//==============================================================================
// Wrist.cpp
//==============================================================================

#include "Wrist.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace RC = RobotConfiguration;

//==============================================================================
// Construction

Wrist::Wrist()  {
    m_wrist_speed_controller = NULL;
    m_wrist_rotation_set_angle = kWristAngleNotSet; // a nonsensical value to indicate we are not trying to drive

    // Under proper match configuration the robot begins with the wrist in the start
    // position. This is not a zero angle according to our convention of 0 degrees
    // being along the arm, but the encode will measure zero here.
    m_wrist_angle_zero_encoder_offset_degrees = kWristStartDegrees;
}

Wrist::~Wrist() {
    Shutdown();
}



//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Wrist::Setup() {
    std::cout << "Wrist::Setup()\n";

    m_wrist_speed_controller = new TalonSRX(RC::kWristTalonId);

    m_wrist_speed_controller->SetInverted(true);

    // Set the wrist current limits
    m_wrist_speed_controller->ConfigPeakCurrentLimit(RC::kWristMotorPeakCurrentLimit, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigPeakCurrentDuration(RC::kWristMotorPeakCurrentDurationMs, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigContinuousCurrentLimit(RC::kWristMotorContinuousCurrentLimit, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive for wrist to 0.4.
    m_wrist_speed_controller->ConfigNominalOutputForward(+0.05, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigNominalOutputReverse(-0.05, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigPeakOutputForward(+1.0, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigPeakOutputReverse(-1.0, RC::kTalonTimeoutMs);

    // Set the speed controller ramp as per the drive base to prevent very sudden
    // control changes. We use the slow driver ramp rate at all times, because
    // that is what we always have used.
    m_wrist_speed_controller->ConfigOpenloopRamp(RC::kWristMotorRampRateDriverS, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->ConfigClosedloopRamp(0.05, RC::kTalonTimeoutMs);

    // The wrist should brake in neutral, otherwise it will fall very quickly (note it
    // still falls slowly).
    m_wrist_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // The wrist uses a CTRE magnetic encoder, which operates in the reverse direction
    // to the motor drive.
    m_wrist_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
														   RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_wrist_speed_controller->SetSensorPhase(false);

	// Initialise the encoder to zero, for testing assume the wrist is pointing up.
    m_wrist_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

	// Setting this flag to true is recommended for operation within a single revolution
	m_wrist_speed_controller->ConfigFeedbackNotContinuous(false, RC::kTalonTimeoutMs);

    // The wrist has one only limit switch for reference position in the full reverse angle
    m_wrist_speed_controller->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated,
    													   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 
    													   RC::kTalonTimeoutMs);

	// Set soft limts in both directions
	m_wrist_speed_controller->ConfigForwardSoftLimitEnable(true, RC::kTalonTimeoutMs);
	int wrist_maximum_angle_encoder = ConvertAngleDegreesToEncoder(kWristMaximumForwardDegrees);
	m_wrist_speed_controller->ConfigForwardSoftLimitThreshold(wrist_maximum_angle_encoder, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->ConfigReverseSoftLimitEnable(true, RC::kTalonTimeoutMs);
	int wrist_minimum_angle_encoder = ConvertAngleDegreesToEncoder(kWristMaximumReverseDegrees);
	m_wrist_speed_controller->ConfigReverseSoftLimitThreshold(wrist_minimum_angle_encoder, RC::kTalonTimeoutMs);


    const double kF = 1.5; // Testing
    const double kP = 1.4;   // Testing
    const double kI = 0.0016;      // Testing
    const double kD = 140;      // Testing

	m_wrist_speed_controller->Config_kF(RC::kTalonRunProfileSlotIdx, kF, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->Config_kP(RC::kTalonRunProfileSlotIdx, kP, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->Config_kI(RC::kTalonRunProfileSlotIdx, kI, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->Config_kD(RC::kTalonRunProfileSlotIdx, kD, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->SelectProfileSlot(RC::kTalonRunProfileSlotIdx, RC::kTalonPidIdx);

	// Set the close loop error. NOTE: This is calculating a delta angle, not an absolute one
	// and hence does not use ConvertAngleDegreesToEncoder().
	const double kAllowableCloseLoopErrorDegrees = 2;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorDegrees / kWristDegreesPerEncoder;
	m_wrist_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	SetupMotionMagic(1.0);
}

void Wrist::Shutdown() {
    std::cout << "Wrist::Shutdown()\n";
}

void Wrist::Periodic(bool show_dashboard)
{
    bool forward_limit = m_wrist_speed_controller->GetSensorCollection().IsFwdLimitSwitchClosed();
	double angle_degrees = GetWristAngleDegrees();
    if (forward_limit) {
        double new_wrist_angle_zero_encoder_offset_degrees = kWristMaximumForwardDegrees - angle_degrees;
 //       std::cout << "Wrist forward limit. Updating offset from " << m_wrist_angle_zero_encoder_offset_degrees <<
 //                    " to " << new_wrist_angle_zero_encoder_offset_degrees << "\n";
        //m_wrist_angle_zero_encoder_offset_degrees = new_wrist_angle_zero_encoder_offset_degrees;
    }

	if (show_dashboard) {
		// Wrist position and motor state
		frc::SmartDashboard::PutNumber("Wrist Angle", angle_degrees);
		frc::SmartDashboard::PutNumber("Wrist Current", m_wrist_speed_controller->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("Wrist Encoder", m_wrist_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
		frc::SmartDashboard::PutNumber("Wrist Motor", m_wrist_speed_controller->GetMotorOutputPercent());
		frc::SmartDashboard::PutBoolean("Wrist FLimit", forward_limit);
	}

	// Log any faults on the talon
	Faults wrist_talon_faults;
	m_wrist_speed_controller->GetFaults(wrist_talon_faults);
	if (wrist_talon_faults.HasAnyFault()) {
		//std::cout << "Wrist Faults " << wrist_talon_faults.ToString() << "\n";
	}
}


//==============================================================================
// Operations

double Wrist::GetWristAngleDegrees() {
	return ConvertEncoderToAngleDegrees(m_wrist_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
}

void Wrist::DriveToAngleDegrees(double angle_degrees, double velocity_factor)
{
	if (angle_degrees < kWristMaximumReverseDegrees) angle_degrees = kWristMaximumReverseDegrees;
	if (angle_degrees > kWristMaximumForwardDegrees) angle_degrees = kWristMaximumForwardDegrees;
    //std::cout << "Setting wrist angle to " << angle_degrees << "  velocity_factor " << velocity_factor << "\n";

    // Setup the velocity and acceleration for Motion Magic
	// Only doing full speed now so do not set up again
    //SetupMotionMagic(velocity_factor);

	// Record the arm extension
	m_wrist_rotation_set_angle = angle_degrees;

	// Convert the angle to encoder units and drive to the position with Motion Magic
    double wrist_angle_native = ConvertAngleDegreesToEncoder(m_wrist_rotation_set_angle); 
	m_wrist_speed_controller->Set(ControlMode::MotionMagic, wrist_angle_native);
}

bool Wrist::IsAngleSet() {
	return (m_wrist_rotation_set_angle != kWristAngleNotSet);
}

void Wrist::ManualDriveWrist(double percentage_speed) {
  	m_wrist_rotation_set_angle = kWristAngleNotSet;
	m_wrist_speed_controller->Set(ControlMode::PercentOutput, percentage_speed*0.15);
}

void Wrist::TestDriveWrist(frc::Joystick* joystick)
{
	// Drive wrist using the left stick

	double wrist_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
    if (fabs(wrist_drive) < RC::kJoystickDeadzone) wrist_drive = 0.0;
    wrist_drive = KoalafiedUtilities::PowerAdjust(wrist_drive, RobotConfiguration::kWristJoystickPower);
	if (wrist_drive != 0.0) {
	    ManualDriveWrist(wrist_drive);
		// std::cout << "Manual drive " << wrist_drive << "\n";
 	} else {
		if (!IsAngleSet()) {
			//ManualDriveWrist(0.0);
			// Lock the wrist at the current position by doing close loop drive to it.
			double angle_degrees = GetWristAngleDegrees();
			DriveToAngleDegrees(angle_degrees, 0.8);
		}
	}

	// Log the F value occasionally
    static int counter = 0;
    if (counter++ % 10 == 0 && fabs(m_wrist_speed_controller->GetMotorOutputVoltage()) > 0.05) {
        KoalafiedUtilities::CalculateAndLogF(m_wrist_speed_controller, kWristDegreesPerEncoder/RC::kTalonTimeBaseS, "Wrist");
    }

    // Close loop test operation
	double velocity_factor = 0.8;
	switch (joystick->GetPOV(0)) {
		case RC::kJoystickPovUp:    DriveToAngleDegrees(15.0, velocity_factor); break;
		case RC::kJoystickPovDown:  DriveToAngleDegrees(-15.0, velocity_factor); break;
		case RC::kJoystickPovRight: DriveToAngleDegrees(45.0, velocity_factor); break;
		case RC::kJoystickPovLeft:  DriveToAngleDegrees(-45.0, velocity_factor); break;
	}
}

void Wrist::TestDriveAngle(ControlMode control_mode, double extension_angle) {
	std::cout << "Test drive wrist to " << extension_angle << "\"\n";
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

	// Base the velocity on a time to travel the full extension distance. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const double kWristFullRotationTimeS = 1.5;
    double wrist_angle_range = kWristMaximumForwardDegrees - kWristMaximumReverseDegrees;
	double velocity_degrees_per_second = velocity_factor * wrist_angle_range/kWristFullRotationTimeS;
	double velocity_native = velocity_degrees_per_second * RC::kTalonTimeBaseS / kWristDegreesPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kWristAccelerationTimeS = 0.5;
	double acceleration_native = velocity_native/kWristAccelerationTimeS; 

	// Set the parameters in the Talon
	m_wrist_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_wrist_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

double Wrist::ConvertAngleDegreesToEncoder(double angle_degrees) {
	return (angle_degrees - m_wrist_angle_zero_encoder_offset_degrees) / kWristDegreesPerEncoder;
}

double Wrist::ConvertEncoderToAngleDegrees(double encoder_position) {
	return encoder_position * kWristDegreesPerEncoder + m_wrist_angle_zero_encoder_offset_degrees;
}
