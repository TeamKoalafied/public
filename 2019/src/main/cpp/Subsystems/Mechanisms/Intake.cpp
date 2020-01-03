//==============================================================================
// Intake.cpp
//==============================================================================

#include "Intake.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;

const double Intake::kIntakeDegreesPerEncoder = 360.0 / (4096.0 * RC::kIntakeRetractRatio);


//==============================================================================
// Construction

Intake::Intake()  {
    m_intake_retract_controller = NULL;
    m_intake_rotation_set_angle = kIntakeAngleNotSet; // a nonsensical value to indicarte we are not trying to drive

    SetIntakeRollerVelocity(kIntakeRollerSpeedDef);

	m_roller_grabbing = false;
	m_roller_current_limit_exceeded = false;
}

Intake::~Intake() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Intake::Setup() {
    std::cout << "Intake::Setup()\n";
    
    m_intake_retract_controller = new TalonSRX(RC::kIntakeRetractTalonId);
    m_intake_roller_controller = new TalonSRX(RC::kIntakeRollerTalonId);

    // Set the intake retract motor current limits
    m_intake_retract_controller->ConfigPeakCurrentLimit(RC::kIntakeRetractMotorPeakCurrentLimit, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigPeakCurrentDuration(RC::kIntakeRetractMotorPeakCurrentDurationMs, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigContinuousCurrentLimit(RC::kIntakeRetractMotorContinuousCurrentLimit, RC::kTalonTimeoutMs);
    m_intake_retract_controller->EnableCurrentLimit(true);

    // Set the intake roller motor current limits
    m_intake_roller_controller->ConfigPeakCurrentLimit(RC::kIntakeRollerMotorPeakCurrentLimit, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigPeakCurrentDuration(RC::kIntakeRollerMotorPeakCurrentDurationMs, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigContinuousCurrentLimit(RC::kIntakeRollerMotorContinuousCurrentLimit, RC::kTalonTimeoutMs);
    m_intake_roller_controller->EnableCurrentLimit(true);

    // Set the nominal (aka minimum) close loop drive
    m_intake_retract_controller->ConfigNominalOutputForward(+0.1, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigNominalOutputReverse(-0.1, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigPeakOutputForward(+1.0, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigPeakOutputReverse(-1.0, RC::kTalonTimeoutMs);

    m_intake_roller_controller->ConfigNominalOutputForward(+0.1, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigNominalOutputReverse(-0.1, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigPeakOutputForward(+1.0, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigPeakOutputReverse(-1.0, RC::kTalonTimeoutMs);

    // Set the speed controller ramp. This is set very short so the arm moves quickly
    m_intake_retract_controller->ConfigOpenloopRamp(0.5, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigClosedloopRamp(0.05, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigOpenloopRamp(0.05, RC::kTalonTimeoutMs);
    m_intake_roller_controller->ConfigClosedloopRamp(0.05, RC::kTalonTimeoutMs);

    // Both motors should brake in neutral
    m_intake_retract_controller->SetNeutralMode(NeutralMode::Brake);
    m_intake_roller_controller->SetNeutralMode(NeutralMode::Brake);

    // The intake retract uses a CTRE magnetic encoder, which operates in the reverse direction
    // to the motor drive.
    m_intake_retract_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_intake_retract_controller->SetSensorPhase(false); // Is reversed

    // Initialise the encoder to zero, because the intakes starts retracted when the robot
    // starts up.
    m_intake_retract_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

    // Setup soft limits in both directions for the retract intake as there are not limit switches
    m_intake_retract_controller->ConfigForwardSoftLimitEnable(true, RC::kTalonTimeoutMs);
    int retract_maximum_forward_extension_encoder = kIntakeForwardLimitDegrees / kIntakeDegreesPerEncoder;
    m_intake_retract_controller->ConfigForwardSoftLimitThreshold(retract_maximum_forward_extension_encoder, RC::kTalonTimeoutMs);
    m_intake_retract_controller->ConfigReverseSoftLimitEnable(true, RC::kTalonTimeoutMs);
    int retract_maximum_reverse_extension_encoder = kIntakeReverseLimitDegrees / kIntakeDegreesPerEncoder;
    m_intake_retract_controller->ConfigReverseSoftLimitThreshold(retract_maximum_reverse_extension_encoder, RC::kTalonTimeoutMs);

    // Setup the close loop PIDF parameters
    const double kF = 1.5;  // Test
    const double kP = 0.8; // Test
    const double kI = 0.0;  // Test
    const double kD = 0.0;  // Test
    // const double kI = 0.0016;  // Test
    // const double kD = 140;  // Test

    m_intake_retract_controller->Config_kF(RC::kTalonRunProfileSlotIdx, kF, RC::kTalonTimeoutMs);
    m_intake_retract_controller->Config_kP(RC::kTalonRunProfileSlotIdx, kP, RC::kTalonTimeoutMs);
    m_intake_retract_controller->Config_kI(RC::kTalonRunProfileSlotIdx, kI, RC::kTalonTimeoutMs);
    m_intake_retract_controller->Config_kD(RC::kTalonRunProfileSlotIdx, kD, RC::kTalonTimeoutMs);
    m_intake_retract_controller->SelectProfileSlot(RC::kTalonRunProfileSlotIdx, RC::kTalonPidIdx);

    // Set the close loop error
    const double kAllowableCloseLoopErrorDegrees = 2;
    double allowable_close_loop_error_encoder = kAllowableCloseLoopErrorDegrees / kIntakeDegreesPerEncoder;
    m_intake_retract_controller->ConfigAllowableClosedloopError(RC::kTalonRunProfileSlotIdx,
		allowable_close_loop_error_encoder, RC::kTalonTimeoutMs);

    // Setup the motion magic parameters for full speed operation
    SetupMotionMagic(1.0);
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic(bool show_dashboard)
{
    // Check the roller current and stop the motors if high current is detected
    // indicating a ball has been grabbed.
	CheckRollerCurrent();

	if (show_dashboard) {
		// Intake position and motor state
		frc::SmartDashboard::PutNumber("Intake Roller Motor", m_intake_roller_controller->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("Intake Roller Current", m_intake_roller_controller->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("Intake Angle Motor", m_intake_retract_controller->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("Intake Angle Current", m_intake_retract_controller->GetOutputCurrent());
		int angle_encoder = m_intake_retract_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
		frc::SmartDashboard::PutNumber("Intake Angle Encoder", angle_encoder);
		double angle_degrees = GetIntakeAngleDegrees();
		frc::SmartDashboard::PutNumber("Intake Angle", angle_degrees);
		Faults talon_faults;
		m_intake_retract_controller->GetFaults(talon_faults);
		frc::SmartDashboard::PutBoolean("Intake Angle RLimit", talon_faults.ReverseLimitSwitch);
		frc::SmartDashboard::PutBoolean("Intake Angle FLimit", talon_faults.ForwardSoftLimit);
	}
}


//==============================================================================
// Operations

Intake::IntakePosition Intake::GetIntakePosition() {
    // Check if the intake is at the correct angle for one of the positions. Test using
    // a generous tolerance as exact positioning is not required.
	double angle_degrees = GetIntakeAngleDegrees();
	if (fabs(angle_degrees - kIntakeInAngleDegrees) < kIntakeToleranceAngleDegrees) return IntakePosition::In;
	if (fabs(angle_degrees - kIntakeVerticalAngleDegrees) < kIntakeToleranceAngleDegrees) return IntakePosition::Vertical;
	if (fabs(angle_degrees - kIntakeOutAngleDegrees) < kIntakeToleranceAngleDegrees) return IntakePosition::Out;

    // If not at any of the positions we must be moving between them
    return IntakePosition::Moving;
}

void Intake::SetIntakePosition(IntakePosition position) {
    // Drive the intake to the angle for the require position
	switch (position) {
		case IntakePosition::In:
			DriveToAngleDegrees(kIntakeInAngleDegrees, 1.0);
			break;
		case IntakePosition::Out:
			DriveToAngleDegrees(kIntakeOutAngleDegrees, 1.0);
			break;
		case IntakePosition::Vertical:
			DriveToAngleDegrees(kIntakeVerticalAngleDegrees, 1.0);
			break;
	}
}

// void Intake::RaiseIntake(double motorspeed) {
// 	m_intake_retract_controller->Set(ControlMode::PercentOutput, motorspeed);
// }

// void Intake::LowerIntake(double motorspeed) {
// 	m_intake_retract_controller->Set(ControlMode::PercentOutput, -motorspeed);
// }

void Intake::StartIntakeRoller(bool reverse) {
	if (m_ball_loaded) {
		std::cout << "ERROR Cannot start the intake when a ball is loaded\n";
	}
    double speed = reverse ? -m_intake_roller_speed : m_intake_roller_speed;
    m_intake_roller_controller->Set(ControlMode::PercentOutput, speed);
	m_roller_grabbing = true;
}

void Intake::StopIntakeRoller() {
    m_intake_roller_controller->Set(ControlMode::PercentOutput, 0.0);
	m_roller_grabbing = false;
}

void Intake::SetIntakeRollerVelocity(double speed) {
	m_intake_roller_speed = speed;
}

double Intake::GetIntakeAngleDegrees() {
    // Convert Encoder Steps to Intake Rotation Angle
    return m_intake_retract_controller->GetSelectedSensorPosition(RC::kTalonPidIdx) * kIntakeDegreesPerEncoder;
}

void Intake::DriveToAngleDegrees(double angle_degrees, double velocity_factor)
{
    // Clampe the angle to the allowed range
	if (angle_degrees < kIntakeReverseLimitDegrees) angle_degrees = kIntakeReverseLimitDegrees;
	if (angle_degrees > kIntakeForwardLimitDegrees) angle_degrees = kIntakeForwardLimitDegrees;
    std::cout << "Setting intake angle to " << angle_degrees << "  velocity_factor " << velocity_factor << "\n";

    // Setup the velocity and acceleration for Motion Magic
	// Only doing full speed now so do not set up again
    //SetupMotionMagic(velocity_factor);

    // Record the arm extension
    m_intake_rotation_set_angle = angle_degrees;

    // Convert the angle to encoder units and drive to the position with Motion Magic
    double intake_angle_native = m_intake_rotation_set_angle / kIntakeDegreesPerEncoder; 
    m_intake_retract_controller->Set(ControlMode::MotionMagic, intake_angle_native);
}

bool Intake::IsAngleSet() {
	return (m_intake_rotation_set_angle != kIntakeAngleNotSet);
}

void Intake::ManualDriveIntake(double percentage_speed) {
  	m_intake_rotation_set_angle = kIntakeAngleNotSet;
	m_intake_retract_controller->Set(ControlMode::PercentOutput, percentage_speed*0.4);
}

void Intake::TestDriveIntake(frc::Joystick* joystick) {

    // Drive the roller and retract from the two joystick Y axes for testing

    double roller_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
    if (fabs(roller_drive) < RC::kJoystickDeadzone) roller_drive = 0.0;
    m_intake_roller_controller->Set(ControlMode::PercentOutput, roller_drive);

    double retract_drive = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(retract_drive) < RC::kJoystickDeadzone) retract_drive = 0.0;
    retract_drive = KoalafiedUtilities::PowerAdjust(retract_drive, RobotConfiguration::kIntakeJoystickPower);
    if (retract_drive != 0.0) {
        ManualDriveIntake(retract_drive);
        std::cout << "Manual drive " << retract_drive << "\n";
    } else {
        if (!IsAngleSet()) {
	        // Lock the intake at the current position by doing close loop drive to it.
	        double angle_degrees = GetIntakeAngleDegrees();
	        DriveToAngleDegrees(angle_degrees, 0.8);
        }
    }

    // Log the F value occasionally
    static int counter = 0;
    if (counter++ % 10 == 0 && fabs(m_intake_retract_controller->GetMotorOutputVoltage()) > 0.05) {
        KoalafiedUtilities::CalculateAndLogF(m_intake_retract_controller, kIntakeDegreesPerEncoder/RC::kTalonTimeBaseS, "Intake");
    }

    // Close loop test operation
    double velocity_factor = 0.8;
    switch (joystick->GetPOV(0)) {
    case RC::kJoystickPovUp:    DriveToAngleDegrees(-68.0, velocity_factor); break;
    case RC::kJoystickPovDown:  DriveToAngleDegrees(-68.0, velocity_factor); break;
    case RC::kJoystickPovRight: DriveToAngleDegrees(5.0, velocity_factor); break;
    case RC::kJoystickPovLeft:  DriveToAngleDegrees(-160.0, velocity_factor); break;
    }
}

//==========================================================================
// Control Implementation

void Intake::CheckRollerCurrent() {
	// Do not apply the current limit unless we are grabbing the ball
	if (!m_roller_grabbing) return;

	if (m_roller_current_limit_exceeded) {
		// If the current limit has been exceeded then check if enough time has passed to
		// stop the rollers and record that a ball is loaded.
		if (m_roller_current_timer.Get() >= RobotConfiguration::kIntakeRollerMotorHighCurrentTimeS) {
			std::cout << "Roller stopped due to current exceeded. Recording ball loaded\n";
			m_ball_loaded = true;
			StopIntakeRoller();
		}
	} else {
		// No high current has been detected so far so get the current now and test if it is
		// high enough to indicate a cube.
		double roller_current = m_intake_roller_controller->GetOutputCurrent();
		if (roller_current >= RobotConfiguration::kIntakeRollerMMotorGrabCurrentA) {
			// The current is high so set a flag and start a timer that will actually
			// stop the motors after a short delay.
			std::cout << "Roller current exceeded " << roller_current << "A\n";
			m_roller_current_limit_exceeded = true;
			m_roller_current_timer.Reset();
			m_roller_current_timer.Start();
		}
	}
}
void Intake::TestDriveAngle(ControlMode control_mode, double extension_angle) {
	std::cout << "Test drive intake to " << extension_angle << "\"\n";
	// Record the intake angle. This stops manual driving from interfering.
	m_intake_rotation_set_angle = extension_angle;

	m_intake_retract_controller->SelectProfileSlot(RC::kTalonTuneProfileSlotIdx, RC::kTalonPidIdx);

	// Calculate the angle in encode units and drive to it
	double intake_rotation_native = m_intake_rotation_set_angle / kIntakeDegreesPerEncoder;
	m_intake_retract_controller->Set(control_mode, intake_rotation_native); 

    std::cout << "intake_rotation_native " << intake_rotation_native <<  "\"\n";
    std::cout << "current_encoder_count " << m_intake_retract_controller->GetSelectedSensorPosition(RC::kTalonPidIdx) <<  "\"\n";
}


//==========================================================================
// Talon Setup

void  Intake::SetupMotionMagic(double velocity_factor) {
	// Clip the velocity factor to the allowed range
	if (velocity_factor <= 0.1) velocity_factor = 0.1;
	if (velocity_factor > 1.0) velocity_factor = 1.0;

	// Base the velocity on a time to travel the full extension distance. Because
	// of acceleration a full traversal will take longer than this.
	// Motion Magic velocity is measured in encoder counts per Talon time base.
	const double kIntakeFullRotationTimeS = 0.75;  // was 1.5
    double intake_angle_range = kIntakeForwardLimitDegrees - kIntakeReverseLimitDegrees;
	double velocity_degrees_per_second = velocity_factor * intake_angle_range/kIntakeFullRotationTimeS;
	double velocity_native = velocity_degrees_per_second * RC::kTalonTimeBaseS / kIntakeDegreesPerEncoder;

	// Base the acceleration on a time to reach maximum velocity.
	// Motion Magic acceleration is measured in velocity units per second.
	const double kIntakeAccelerationTimeS = 0.2;  // was 0.5
	double acceleration_native = velocity_native/kIntakeAccelerationTimeS; 

	// Set the parameters in the Talon
	m_intake_retract_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
	m_intake_retract_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

