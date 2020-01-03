//==============================================================================
// Pivot.cpp
//==============================================================================

#include "Pivot.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"
#include "../../PeriodicTimer.h"
        
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

namespace RC = RobotConfiguration;

const double Pivot::kPivotDegreesPerEncoder = 360.0 / (4096.0 * RC::kPivotGearRatio);


//==============================================================================
// Construction

Pivot::Pivot() {
    m_pivot_speed_controller = NULL;
    m_pivot_rotation_set_angle = kPivotAngleNotSet; // a nonsensical value to indicate we are not trying to drive

    // Under proper match configuration the robot begins with the arm in the start
    // position. This is not a zero angle according to our convention of 0 degrees
    // being upwards, but the encode will measure zero here.
    m_pivot_angle_zero_encoder_offset_degrees = kPivotStartDegrees;

	// The limit switches are not triggered in the start position
	m_forward_limit = false;
	m_reverse_limit = false;
}

Pivot::~Pivot() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Pivot::Setup() {
    std::cout << "Pivot::Setup()\n";

    m_pivot_speed_controller = new TalonSRX(RC::kPivotTalonId);

    m_pivot_speed_controller->SetInverted(true);

    // Set the pivot current limits
    m_pivot_speed_controller->ConfigPeakCurrentLimit(RC::kPivotMotorPeakCurrentLimit, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigPeakCurrentDuration(RC::kPivotMotorPeakCurrentDurationMs, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigContinuousCurrentLimit(RC::kPivotMotorContinuousCurrentLimit, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->EnableCurrentLimit(true);

    // TODO Review nominal and peak output for pivot.
    m_pivot_speed_controller->ConfigNominalOutputForward(+0.1, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigNominalOutputReverse(-0.1, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigPeakOutputForward(+1.0f, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigPeakOutputReverse(-1.0f, RC::kTalonTimeoutMs);

    // Set the speed controller ramp. This is set very short so the pivot moves quickly.
    m_pivot_speed_controller->ConfigOpenloopRamp(RC::kPivotMotorOpenLoopRampRateS, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigClosedloopRamp(RC::kPivotMotorClosedLoopRampRateS, RC::kTalonTimeoutMs);

	// The pivot should brake in neutral.
    m_pivot_speed_controller->SetNeutralMode(NeutralMode::Brake);

    m_pivot_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->SetSensorPhase(true);

    // Initialise the encoder to zero - for testing we assume the arm is pointing straight up.
    m_pivot_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

	// Pivot limit switch settings
	m_pivot_speed_controller->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
														   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 
														   RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
														   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 
														   RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigForwardSoftLimitEnable(true, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigReverseSoftLimitEnable(true, RC::kTalonTimeoutMs);

	// Start with the pivot set to full range (otherwise an old restricted range may apply)
    SetSoftLimitsAngleDegrees(RC::kPivotMaximumForwardDegrees, RC::kPivotMaximumReverseDegrees);

//	const double kF = 2.5; // Testing
	const double kF = 0.0; // Testing
	const double kP = 0.64; // Testing was 0.32 first comp
	const double kI = 0.0; // Testing
	const double kD = 0.0; // Testing

	m_pivot_speed_controller->Config_kF(RC::kTalonRunProfileSlotIdx, kF, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->Config_kP(RC::kTalonRunProfileSlotIdx, kP, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->Config_kI(RC::kTalonRunProfileSlotIdx, kI, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->Config_kD(RC::kTalonRunProfileSlotIdx, kD, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->SelectProfileSlot(RC::kTalonRunProfileSlotIdx, RC::kTalonPidIdx);

	// Set the close loop error. NOTE: This is calculating a delta angle, not an absolute one
	// and hence does not use ConvertAngleDegreesToEncoder().
	const double kAllowableCloseLoopErrorDegrees = 1;
	double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorDegrees / kPivotDegreesPerEncoder;
	m_pivot_speed_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

	// Setup the motion magic parameters for full speed operation
	SetupMotionMagic(1.0);
}

void Pivot::Shutdown() {
    std::cout << "Pivot::Shutdown()\n";
}

void Pivot::Periodic(bool show_dashboard) {
	double angle_degrees = GetPivotAngleDegrees();

	// When we first hit a limit switch see how well the encoder offset is set
	bool forward_limit = m_pivot_speed_controller->GetSensorCollection().IsFwdLimitSwitchClosed();
    if (forward_limit && !m_forward_limit) {
        double new_pivot_angle_zero_encoder_offset_degrees = RC::kPivotMaximumForwardDegrees - angle_degrees;
        std::cout << "Pivot forward limit. Updating offset from " << m_pivot_angle_zero_encoder_offset_degrees <<
                     " to " << new_pivot_angle_zero_encoder_offset_degrees << "\n";
        //m_pivot_angle_zero_encoder_offset_degrees = new_pivot_angle_zero_encoder_offset_degrees;
    }
	m_forward_limit = forward_limit;

	bool reverse_limit = m_pivot_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
    if (reverse_limit && !m_reverse_limit) {
        double new_pivot_angle_zero_encoder_offset_degrees = RC::kPivotMaximumReverseDegrees - angle_degrees;
        std::cout << "Pivot backward limit. Updating offset from " << m_pivot_angle_zero_encoder_offset_degrees <<
                     " to " << new_pivot_angle_zero_encoder_offset_degrees << "\n";
        //m_pivot_angle_zero_encoder_offset_degrees = new_pivot_angle_zero_encoder_offset_degrees;
    }
	m_reverse_limit = reverse_limit;

	if (show_dashboard) {
		frc::SmartDashboard::PutNumber("Pivot Current", m_pivot_speed_controller->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("Pivot Encoder", m_pivot_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
		frc::SmartDashboard::PutNumber("Pivot Motor", m_pivot_speed_controller->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("Pivot Angle", angle_degrees);
		frc::SmartDashboard::PutBoolean("Pivot FLimit", forward_limit);
		frc::SmartDashboard::PutBoolean("Pivot RLimit", reverse_limit);
	}
}


//==============================================================================
// Operations

double Pivot::GetPivotAngleDegrees() {
   	// Get the angle in encoder units and convert to degrees
	int angle_encoder = m_pivot_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	double angle_degrees = ConvertEncoderToAngleDegrees(angle_encoder);
	return angle_degrees;
}

void Pivot::DriveToAngleDegrees(double angle_degrees, double velocity_factor) {
	if (angle_degrees < RC::kPivotMaximumReverseDegrees) angle_degrees = RC::kPivotMaximumReverseDegrees;
	if (angle_degrees > RC::kPivotMaximumForwardDegrees) angle_degrees = RC::kPivotMaximumForwardDegrees;
    // std::cout << "Setting pivot angle to " << angle_degrees << "  velocity_factor " << velocity_factor << "\n";

    // Setup the velocity and acceleration for Motion Magic
	// Only doing full speed now so do not set up again
    //SetupMotionMagic(velocity_factor);

	// Record the arm extension
	m_pivot_rotation_set_angle = angle_degrees;

	// Convert the angle to encoder units and drive to the position with Motion Magic
    double pivot_angle_native = ConvertAngleDegreesToEncoder(m_pivot_rotation_set_angle); 
	m_pivot_speed_controller->Set(ControlMode::Position, pivot_angle_native);
//	m_pivot_speed_controller->Set(ControlMode::MotionMagic, pivot_angle_native);
	std::cout << "Pivot Drive to Angle " << angle_degrees << "\n";
}

bool Pivot::IsAngleSet() {
	return (m_pivot_rotation_set_angle != kPivotAngleNotSet);
}

void Pivot::ManualDrivePivot(double percentage_output) {
	if (m_pivot_rotation_set_angle != kPivotAngleNotSet) std::cout << "Pivot Manual Drive " << percentage_output << "\n";
	m_pivot_rotation_set_angle = kPivotAngleNotSet;
    m_pivot_speed_controller->Set(ControlMode::PercentOutput, percentage_output*RC::kPivotJoystickMax);
}

void Pivot::SetSoftLimitsAngleDegrees(double forward_limit_degrees, double reverse_limit_degrees) {
	double angle_degrees = GetPivotAngleDegrees();
	// if (angle_degrees > forward_limit_degrees || angle_degrees < reverse_limit_degrees) {
	//	std::cout << "Current pivot angle " << angle_degrees << " outside software limits " << forward_limit_degrees <<  " to " << reverse_limit_degrees << "\n";
	// }
//	static PeriodicTimer timer("Pivot");
//	timer.PeriodicStart();
	static double prev_forward_limit_degrees = kPivotAngleNotSet;
	static double prev_reverse_limit_degrees = kPivotAngleNotSet;

	if (prev_forward_limit_degrees != forward_limit_degrees) {
		m_pivot_speed_controller->ConfigForwardSoftLimitEnable(true, RC::kTalonTimeoutMs);
		int pivot_maximum_angle_encoder = ConvertAngleDegreesToEncoder(forward_limit_degrees);
		m_pivot_speed_controller->ConfigForwardSoftLimitThreshold(pivot_maximum_angle_encoder, RC::kTalonTimeoutMs);
		prev_forward_limit_degrees = forward_limit_degrees;
	}

	if (prev_reverse_limit_degrees != reverse_limit_degrees) {
		m_pivot_speed_controller->ConfigReverseSoftLimitEnable(true, RC::kTalonTimeoutMs);
		int pivot_minimum_angle_encoder = ConvertAngleDegreesToEncoder(reverse_limit_degrees);
		m_pivot_speed_controller->ConfigReverseSoftLimitThreshold(pivot_minimum_angle_encoder, RC::kTalonTimeoutMs);
		prev_reverse_limit_degrees = reverse_limit_degrees;
	}

//	timer.PeriodicEnd();
}

bool Pivot::GetForwardSoftLimitHit() {
	Faults pivot_talon_faults;
	m_pivot_speed_controller->GetFaults(pivot_talon_faults);
	return pivot_talon_faults.ForwardSoftLimit;
}

bool Pivot::GetReverseSoftLimitHit() {
	Faults pivot_talon_faults;
	m_pivot_speed_controller->GetFaults(pivot_talon_faults);
	return pivot_talon_faults.ReverseSoftLimit;
}

void Pivot::TestDrivePivot(frc::Joystick* joystick) {
    // Drive the pivot using the right Y axis
    double pivot_drive = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(pivot_drive) < RC::kJoystickDeadzone) pivot_drive = 0.0;
    pivot_drive = KoalafiedUtilities::PowerAdjust(pivot_drive, RobotConfiguration::kPivotJoystickPower);
	if (pivot_drive != 0.0) {
	    ManualDrivePivot(pivot_drive);
		// std::cout << "Manual drive " << pivot_drive << "\n";
 	} else {
        if (!IsAngleSet()) {
	        //ManualDrivePivot(0.0);
            //std::cout << "Zeroing drive " << "\n";

			// Lock the pivot at the current position by doing close loop drive to it.
			double angle_degrees = GetPivotAngleDegrees();
			DriveToAngleDegrees(angle_degrees, 0.8);
        }
	}

	// Log the F value occasionally
    static int counter = 0;
    if (counter++ % 10 == 0 && fabs(m_pivot_speed_controller->GetMotorOutputVoltage()) > 0.05) {
        KoalafiedUtilities::CalculateAndLogF(m_pivot_speed_controller, kPivotDegreesPerEncoder/RC::kTalonTimeBaseS, "Pivot");
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

void Pivot::TestDriveAngle(ControlMode control_mode, double extension_angle) {
	std::cout << "Test drive pivot to " << extension_angle << "\"\n";
	// Record the pivot angle. This stops manual driving from interfering.
	m_pivot_rotation_set_angle = extension_angle;

	m_pivot_speed_controller->SelectProfileSlot(RC::kTalonTuneProfileSlotIdx, RC::kTalonPidIdx);

	// Calculate the angle in encode units and drive to it
	double pivot_rotation_native = m_pivot_rotation_set_angle / kPivotDegreesPerEncoder;
	m_pivot_speed_controller->Set(control_mode, pivot_rotation_native); 

    std::cout << "pivot_rotation_native " << pivot_rotation_native <<  "\"\n";
    std::cout << "current_encoder_count " << m_pivot_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx) <<  "\"\n";
}


//==========================================================================
// Talon Setup

void  Pivot::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// Base the velocity on a time to travel the full extension distance. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const double kPivotFullRotationTimeS = 2.5; // was 3.0
    double pivot_angle_range = RC::kPivotMaximumForwardDegrees - RC::kPivotMaximumReverseDegrees;
	double velocity_degrees_per_second = velocity_factor * pivot_angle_range/kPivotFullRotationTimeS;
	double velocity_native = velocity_degrees_per_second * RC::kTalonTimeBaseS / kPivotDegreesPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kPivotAccelerationTimeS = 0.5; // was 1.0
	double acceleration_native = velocity_native/kPivotAccelerationTimeS; 

	// Set the parameters in the Talon
	m_pivot_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_pivot_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

double Pivot::ConvertAngleDegreesToEncoder(double angle_degrees) {
	return (angle_degrees - m_pivot_angle_zero_encoder_offset_degrees) / kPivotDegreesPerEncoder;
}

double Pivot::ConvertEncoderToAngleDegrees(double encoder_position) {
	return encoder_position * kPivotDegreesPerEncoder + m_pivot_angle_zero_encoder_offset_degrees;
}
