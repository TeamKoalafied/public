//==============================================================================
// Pivot.cpp
//==============================================================================

#include "Pivot.h"

#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <iostream>
#include <iostream>

namespace RC = RobotConfiguration;

//const double Pivot::kPivotDegreesPerEncoder = 360.0 / (4096.0 * RC::kPivotRetractRatio);


//==============================================================================
// Construction

Pivot::Pivot()  {
    m_pivot_speed_controller = NULL;
    m_pivot_rotation_set_angle = kPivotAngleNotSet;
}

Pivot::~Pivot() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Pivot::Setup() {
    std::cout << "Pivot::Setup()\n";

    m_pivot_speed_controller = new TalonSRX(RobotConfiguration::kPivotTalonId);

    TalonSRXConfiguration pivot_configuration;

    pivot_configuration.continuousCurrentLimit = 10;
    pivot_configuration.peakCurrentLimit = 10;
    pivot_configuration.peakCurrentDuration = 500;

    // Never run the Pivot motor above 50% as it starts to skip encoder counts
    pivot_configuration.peakOutputForward = 0.5;
    pivot_configuration.peakOutputReverse = -0.5;

    // Motion magic parameters- Slot 0. F and P come from measurement with TestDrivePivot()
    pivot_configuration.slot0.kF = 25.0;
    pivot_configuration.slot0.kP = 56.0;

    // Position control parameters - Slot 1. NOTE: F must be zero or it will drive straight
    // past the set point.
    pivot_configuration.slot1.kF = 0.0;
    pivot_configuration.slot1.kP = 18.0;

    // The pivot has a limit switch in each direction
    pivot_configuration.forwardLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
    pivot_configuration.reverseLimitSwitchNormal = LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
    
    // The JE motor has its own quadrature encoder
    pivot_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
 
    // The forward direction is down and we want to reset the encoder position their because that is
    // where the pivot sits when not in use
    pivot_configuration.clearPositionOnLimitF = true;

    // Do all configuration and log if it fails
    int error = m_pivot_speed_controller->ConfigAllSettings(pivot_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the Pivot Talon failed with code:  " << error << "\n";
    }
    
    // Perform non-configuration setup
    m_pivot_speed_controller->SetSensorPhase(true); // Not reversed
    m_pivot_speed_controller->EnableCurrentLimit(true);
	m_pivot_speed_controller->SetNeutralMode(NeutralMode::Brake);
}

void Pivot::Shutdown() {
    std::cout << "Pivot::Shutdown()\n";
}

void Pivot::Periodic() {
}

//==========================================================================
// State

units::ampere_t Pivot::GetCurrent() const {
    return units::ampere_t(m_pivot_speed_controller->GetOutputCurrent());
}

double Pivot::GetOutput() const {
    return m_pivot_speed_controller->GetMotorOutputPercent();
}

units::degree_t Pivot::GetAngle() const {
    return ConvertEncoderToAngleDegrees(m_pivot_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx));
}

bool Pivot::GetBottomLimitSwitch() const {
    return m_pivot_speed_controller->IsFwdLimitSwitchClosed();
}

bool Pivot::GetTopLimitSwitch() const {
    return m_pivot_speed_controller->IsRevLimitSwitchClosed();
}

//==============================================================================
// Operations

void Pivot::DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor, DriveToMethod drive_to_method)
{
    // Clamp the angle to the allowed range
	if (angle_degrees < kPivotMinimumAngle) angle_degrees = kPivotMinimumAngle;
	if (angle_degrees > kPivotMaximumAngle) angle_degrees = kPivotMaximumAngle;

    // Setup the velocity and acceleration for Motion Magic
    SetupMotionMagic(velocity_factor);

	// Record the pivot angle
	m_pivot_rotation_set_angle = angle_degrees;

	// Convert the angle to encoder units and drive to the position with Motion Magic
    double pivot_angle_native = ConvertAngleDegreesToEncoder(m_pivot_rotation_set_angle); 

    // If the drive method is 'Blend' determine if we should currently be using Position
    // or MotionMagic control.
    if (drive_to_method == DriveToMethod::Blend) {
        // Get the current angle and determine the error angle to the desired angle
        units::degree_t current_angle = GetAngle();
        units::degree_t angle_error = units::math::abs(m_pivot_rotation_set_angle - current_angle);

        // Use MotionMagic when far from the desired angle and Position control when close
        drive_to_method = (angle_error > 2_deg) ? DriveToMethod::MotionMagic : DriveToMethod::Position;
    }

    // Do MotionMagic or Position control as desired, with a different PID slot for each
    switch (drive_to_method) {
        case DriveToMethod::MotionMagic:
            m_pivot_speed_controller->SelectProfileSlot(0, RC::kTalonPidIdx);
            m_pivot_speed_controller->Set(ControlMode::MotionMagic, pivot_angle_native);
            break;
        case DriveToMethod::Position:
            m_pivot_speed_controller->SelectProfileSlot(1, RC::kTalonPidIdx);
            m_pivot_speed_controller->Set(ControlMode::Position, pivot_angle_native);
            break;
        default:
            assert(false);
            m_pivot_speed_controller->Set(ControlMode::PercentOutput, 0.0);
            break;
    }
}


//==========================================================================
// Testing

void Pivot::ManualDrivePivot(double percentage_output) {
    m_pivot_rotation_set_angle = kPivotAngleNotSet;
    m_pivot_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Pivot::TestDrivePivot(frc::XboxController* controller) {
    // By default using the Blend method of control, but use Position if the A button
    // is held down and MotionMagic if the B button is held down. 
    DriveToMethod method = DriveToMethod::Blend;
    if (controller->GetAButton()) {
        method = DriveToMethod::Position;
    } else if (controller->GetBButton()) {
        method = DriveToMethod::MotionMagic;
    }

    // Go to set angles if the POV is pressed, otherwise use manual joystick control
	switch (controller->GetPOV(0)) {
		case RC::kJoystickPovUp:    DriveToAngleDegrees(30.0_deg, 1.0, method); break;
		case RC::kJoystickPovRight: DriveToAngleDegrees(40.0_deg, 1.0, method); break;
		case RC::kJoystickPovDown:  DriveToAngleDegrees(50.0_deg, 1.0, method); break;
		case RC::kJoystickPovLeft:  DriveToAngleDegrees(60.0_deg, 1.0, method); break;
   		default: {
            // JE Motor maximum speed is 420rpm
            double MAX_RPM = 420.0;

            // Use the right joystick Y axis to control the speed of the Pivot. Do closed loop if the
            // left trigger button is held down.
			double pivot_drive = 0.5 * frc::ApplyDeadband(controller->GetRightY(), RC::kJoystickDeadzone);
            bool close_loop = controller->GetLeftBumper();
            KoalafiedUtilities::TuneDriveTalon(m_pivot_speed_controller, "Pivot", pivot_drive, MAX_RPM, close_loop, 44.4 * 4.0);
        }
	}
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

    // Base the velocity on a time to travel the full extension distance. Because
    // of acceleration a full traversal will take longer than this.
    // Motion Magic velocity is measured in encoder counts per Talon time base.
    const units::second_t kPivotFullRotationTimeS = 0.75_s;
    constexpr units::degree_t pivot_angle_range = kPivotMaximumAngle - kPivotMinimumAngle;
    units::degrees_per_second_t velocity_degrees_per_second = velocity_factor * pivot_angle_range/kPivotFullRotationTimeS;
    double velocity_native = velocity_degrees_per_second * units::second_t(RC::kTalonTimeBaseS) / kPivotDegreesPerEncoder;
    std::cout << "Pivot cruise velocity native " << velocity_native << "\n";

    // Base the acceleration on a time to reach maximum velocity.
    // Motion Magic acceleration is measured in velocity units per second.
    const double kPivotAccelerationTimeS = 0.125;
    double acceleration_native = velocity_native/kPivotAccelerationTimeS; 

    // Set the parameters in the Talon
    m_pivot_speed_controller->ConfigMotionCruiseVelocity(velocity_native, RC::kTalonTimeoutMs);
    m_pivot_speed_controller->ConfigMotionAcceleration(acceleration_native, RC::kTalonTimeoutMs);
}

double Pivot::ConvertAngleDegreesToEncoder(units::degree_t angle_degrees) {
	return -(angle_degrees - kPivotZeroAngle) / kPivotDegreesPerEncoder;
}

units::degree_t Pivot::ConvertEncoderToAngleDegrees(units::scalar_t encoder_position) const {
	return -encoder_position * kPivotDegreesPerEncoder + kPivotZeroAngle;
}