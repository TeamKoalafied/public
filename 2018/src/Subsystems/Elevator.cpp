//==============================================================================
// Elevator.cpp
//==============================================================================

#include "Elevator.h"

#include "../RobotConfiguration.h"
#include "../Utilities.h"
#include "../Commands/OperateWithJoystick.h"
#include "../Commands/DriveArm.h"
#include "../Commands/EjectClaw.h"
#include "WPILib.h"

#include <Talon.h>
#include <SmartDashboard/SmartDashboard.h>
#include <iostream>

#include "../Commands/DriveElevator.h"

//==============================================================================
// Construction

Elevator::Elevator() :
	TSingleton<Elevator>(this),
	frc::Subsystem("Elevator") {

	m_lift_speed_controller = NULL;
	m_operator_joystick = NULL;

    m_roller_state = RollerState::Stopped;
    m_cube_loaded = false;
	m_roller_current_limit_exceeded = false;
	m_roller_timeout_s = -1;

	m_arm_up_command = NULL;
	m_arm_down_command = NULL;

	m_grab_trigger_pressed = false;
	m_eject_trigger_pressed = false;

	m_log_counter = 0;

	m_lift_position = LiftPosition::kIntermediate;

    m_high_lift_slow_enabled = true;
}

Elevator::~Elevator() {

}


//==============================================================================
// frc::Subsystem Function Overrides

void Elevator::InitDefaultCommand() {
    SetDefaultCommand(new OperateWithJoystick());
}

void Elevator::Periodic() {

	// If either of the trigger buttons is down start the default command. This means that
	// the joystick can be used to cancel any autonomous command, in case something going wrong.
	if (m_operator_joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton) ||
		m_operator_joystick->GetRawButton(RobotConfiguration::kJoystickRTrigButton)) {
		if (!GetDefaultCommand()->IsRunning()) {
			GetDefaultCommand()->Start();
		}
	}

	// Check the roller current (use to detect that a cube is captured) and the roller
	// timeout (used to stop the rollers a set time after a operator command).
    CheckRollerCurrent();
    CheckRollerTimeout();

    // Drive the lift towards a position if one is set
	DriveLiftToPosition();

    // Cube state
	SmartDashboard::PutBoolean("Cube Loaded", m_cube_loaded);

	// Claw state
	SmartDashboard::PutBoolean("Claw Open", IsClawOpen());
	SmartDashboard::PutNumber("Claw Current", (m_left_claw_speed_controller->GetOutputCurrent() +
											   m_right_claw_speed_controller->GetOutputCurrent()) / 2.0);

	// Arm state
	SmartDashboard::PutBoolean("Arm Up", IsArmUp());

	// Lift position and motor state
	LiftPosition lift_position = GetLiftPosition();
	SmartDashboard::PutBoolean("Lift Floor", lift_position == LiftPosition::kFloor);
	SmartDashboard::PutBoolean("Lift Switch", lift_position == LiftPosition::kSwitch);
	SmartDashboard::PutBoolean("Lift Scale", lift_position == LiftPosition::kScale);
	SmartDashboard::PutNumber("Lift Height", GetLiftHeightInch());
	SmartDashboard::PutNumber("Lift Current", m_lift_speed_controller->GetOutputCurrent());
	SmartDashboard::PutNumber("Lift Encoder", m_lift_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx));
	SmartDashboard::PutNumber("Lift Motor", m_lift_speed_controller->GetMotorOutputPercent());
	Faults lift_talon_faults;
	m_lift_speed_controller->GetFaults(lift_talon_faults);
	SmartDashboard::PutBoolean("Lift BLimit", lift_talon_faults.ReverseLimitSwitch);
	SmartDashboard::PutBoolean("Lift TLimit", lift_talon_faults.ForwardLimitSwitch);

    SmartDashboard::PutBoolean("Operate Joystick", GetDefaultCommand()->IsRunning());

	// Any time we hit the bottom limit switch zero the encoder
	if (lift_talon_faults.ReverseLimitSwitch) {
		std::cout << "Zeroing encoder due to hitting bottom limit switch\n";
	    m_lift_speed_controller->SetSelectedSensorPosition(0, kPidDefaultIdx, kTalonElevatorTimeoutMs);
	}
}


//==============================================================================
// Setup and Shutdown

void Elevator::Setup() {
	m_lift_speed_controller = new TalonSRX(RobotConfiguration::kLiftTalonId);

	// The lift has normally open limit switches in both directions
	m_lift_speed_controller->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, kTalonElevatorTimeoutMs);

	// Set the lift current limits
	m_lift_speed_controller->ConfigPeakCurrentLimit(RobotConfiguration::kLiftMotorPeakCurrentLimit, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->ConfigPeakCurrentDuration(RobotConfiguration::kLiftMotorPeakCurrentDurationMs, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->ConfigContinuousCurrentLimit(RobotConfiguration::kLiftMotorContinuousCurrentLimit, kTalonElevatorTimeoutMs);

	// The lift should brake in neutral, otherwise it will fall very quickly (note it
	// still falls slowly).
	m_lift_speed_controller->SetNeutralMode(NeutralMode::Brake);

	// The lift uses a CTRE magnetic encoder, which operates in the reverse direction
	// to the motor drive.
	m_lift_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, kPidDefaultIdx, kTalonElevatorTimeoutMs);
    m_lift_speed_controller->SetSensorPhase(false); // Is reversed

    // Initialise the encode to zero, because the lift starts at the bottom when the robot
    // starts up.
    m_lift_speed_controller->SetSelectedSensorPosition(0, kPidDefaultIdx, kTalonElevatorTimeoutMs);

    // Set the nominal (aka minimum) close loop drive for lifting to 0.4.
    // This is tested as the minimum drive that will actually move the lift upwards.
    // Set the peak close loop drive for dropping to 0.4 as driving down anymore
    // than this is dangerously fast.
    m_lift_speed_controller->ConfigNominalOutputForward(+0.40f, kTalonElevatorTimeoutMs);
    m_lift_speed_controller->ConfigNominalOutputReverse(-0.0f, kTalonElevatorTimeoutMs);
    m_lift_speed_controller->ConfigPeakOutputForward(+1.0f, kTalonElevatorTimeoutMs);
    m_lift_speed_controller->ConfigPeakOutputReverse(-0.4f, kTalonElevatorTimeoutMs);

    // Set the speed controller ramp as per the drive base to prevent very sudden
    // control changes. We use the slow driver ramp rate at all times, because
    // that is what we always have used.
    m_lift_speed_controller->ConfigOpenloopRamp(RobotConfiguration::kDriveMotorRampRateDriverS, kTalonElevatorTimeoutMs);
    m_lift_speed_controller->ConfigClosedloopRamp(RobotConfiguration::kDriveMotorRampRateDriverS, kTalonElevatorTimeoutMs);


	const double kF = 0.0;
	const double kP = 0.25;
	const double kI = 0.0;
	const double kD = 0.0;
	m_lift_speed_controller->SelectProfileSlot(kRunProfileSlotIdx, kPidDefaultIdx);
	m_lift_speed_controller->Config_kF(kRunProfileSlotIdx, kF, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->Config_kP(kRunProfileSlotIdx, kP, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->Config_kI(kRunProfileSlotIdx, kI, kTalonElevatorTimeoutMs);
	m_lift_speed_controller->Config_kD(kRunProfileSlotIdx, kD, kTalonElevatorTimeoutMs);


	m_left_claw_speed_controller = new TalonSRX(RobotConfiguration::kGripperLeftTalonId);
	m_right_claw_speed_controller = new TalonSRX(RobotConfiguration::kGripperRightTalonId);

    // Claw roller current limits
    m_left_claw_speed_controller->ConfigPeakCurrentLimit(RobotConfiguration::kClawMotorPeakCurrentLimitA, kTalonElevatorTimeoutMs);
    m_left_claw_speed_controller->ConfigPeakCurrentDuration(RobotConfiguration::kClawMotorPeakCurrentDurationMs, kTalonElevatorTimeoutMs);
    m_left_claw_speed_controller->ConfigContinuousCurrentLimit(RobotConfiguration::kClawMotorContinuousCurrentLimitA, kTalonElevatorTimeoutMs);
    m_right_claw_speed_controller->ConfigPeakCurrentLimit(RobotConfiguration::kClawMotorPeakCurrentLimitA, kTalonElevatorTimeoutMs);
    m_right_claw_speed_controller->ConfigPeakCurrentDuration(RobotConfiguration::kClawMotorPeakCurrentDurationMs, kTalonElevatorTimeoutMs);
    m_right_claw_speed_controller->ConfigContinuousCurrentLimit(RobotConfiguration::kClawMotorContinuousCurrentLimitA, kTalonElevatorTimeoutMs);


	m_operator_joystick = new Joystick(RobotConfiguration::kJoystickPortOperator);

	// Setup the arm and claw solenoids
	m_arm_solenoid = new frc::Solenoid(RobotConfiguration::kPneumaticsControlModuleId,
									   RobotConfiguration::kPneumaticsArmSolenoidId);
	m_claw_solenoid = new frc::Solenoid(RobotConfiguration::kPneumaticsControlModuleId,
										RobotConfiguration::kPneumaticsClawSolenoidId);

	// Create various commands
	m_elevator_floor_command = new DriveElevator(LiftPosition::kFloor, true);
	m_elevator_switch_command = new DriveElevator(LiftPosition::kSwitch, true);
	m_elevator_scale_command = new DriveElevator(LiftPosition::kScale, true);

	m_arm_up_command = new DriveArm(true);
	m_arm_down_command = new DriveArm(true);

	m_eject_cube_command = new EjectClaw();

	// Initialise the lift to the 'intermediate' position, meaning that it is not
	// being driven to any particular height
	m_lift_position = LiftPosition::kIntermediate;
}

void Elevator::ClearState() {
	Elevator::GetInstance().MoveLiftToPosition(Elevator::LiftPosition::kFloor);
	m_claw_solenoid->Set(false);
	m_arm_solenoid->Set(false);
}

void Elevator::Shutdown() {
	delete m_lift_speed_controller;
	m_lift_speed_controller = NULL;
	delete m_operator_joystick;
	m_operator_joystick = NULL;
}


//==========================================================================
// Operation

void Elevator::ResetJoystickState() {
	// Clear the joystick state, otherwise press and release events that
	// have already occurred my be incorrectly reported.
	KoalafiedUtilities::ClearJoystickButtonState(m_operator_joystick);

	m_grab_trigger_pressed = false;
	m_eject_trigger_pressed = false;
}

void Elevator::DoOperatorDrive() {

	// The left trigger controls grabbing the cube. When the trigger is pulled the
	// claw is opened and the motor run for grabbing, with no timeout. When the trigger is
	// is released the claw closes and runs for another length of time. Note that
	// the roller motor always stops if high current is detected, indicating a cube
	// has been grabbed.
    double left_trigger = m_operator_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftTriggerAxis);
    double right_trigger = m_operator_joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis);
    bool new_grab_trigger_pressed = left_trigger > 0.5 && left_trigger > 2*right_trigger;
    if (new_grab_trigger_pressed && !m_grab_trigger_pressed) {
    	if (!m_cube_loaded) {
			OpenClaw();
			RunClawRollerGrab(-1);
    	} else {
    		std::cout << "Not grabbing because we already have a cube";
    	}
    }
    if (!new_grab_trigger_pressed && m_grab_trigger_pressed) {
    	// The trigger has been released so close the claw and run the roller unless
    	// we already have a cube.
		CloseClaw();
    	if (!m_cube_loaded) {
			RunClawRollerGrab(RobotConfiguration::kClawMotorReleaseButtonTimeS);
    	}
    }
	m_grab_trigger_pressed = new_grab_trigger_pressed;

    // The right trigger controls ejecting the cube. The eject starts when it is pressed
	// and stops when it is released.
    bool new_eject_trigger_pressed = right_trigger > 0.5 && right_trigger > 2*left_trigger;;
    if (new_eject_trigger_pressed && !m_eject_trigger_pressed) {
    	RunClawRollerEject(-1,false); // No timeout
    }
    if (!new_eject_trigger_pressed && m_eject_trigger_pressed) {
    	StopClawRollers();
    }
	m_eject_trigger_pressed = new_eject_trigger_pressed;

//	if (new_grab_trigger_pressed && new_eject_trigger_pressed) {
//		std::cout << "WARNING: Both operator triggers pressed\n";
//	}

    // Arm control. Y (top button) is arm up and A (bottom button) is arm down.
    if (m_operator_joystick->GetRawButton(RobotConfiguration::kJoystickYButton)) {
    	// Use a command that handles making sure the lift is slightly raised before
    	// the arm is raised, so that the cube does not catch on the drive base perimeter.
    	m_arm_up_command->Start();
    }
    if (m_operator_joystick->GetRawButton(RobotConfiguration::kJoystickAButton)) {
    	DropArm();
    }

    if (m_operator_joystick->GetRawButton(RobotConfiguration::kJoystickXButton)) {
    	RunClawRollerGrab(1);
    }


    // The lift input comes from the y axis of the left joystick.
    // Apply a 'deadzone' so that very small inputs all map to 0. This is
    // necessary because the joystick does not centre perfectly when released.
    double lift = -m_operator_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis);
    bool lift_deadzone = fabs(lift) < RobotConfiguration::kJoystickDeadzone;
    if (lift_deadzone) lift = 0.0;

    // Only drive the lift if outside the deadzone, or if the position is not set. This means
    // that if a position is set it will stay set until the driver uses the joystick to move it.
    if (!lift_deadzone || m_lift_position == LiftPosition::kIntermediate) {
    	SmartDashboard::PutNumber("Lift Drive", lift);
        DoLiftDrive(lift);
    }


    //--------------------------------------------------------------------------
    // Test Controls

    if (m_operator_joystick->GetPOV(0) == 0) {
    	m_elevator_scale_command->Start();
    }
    if (m_operator_joystick->GetPOV(0) == 270) {
    	m_elevator_switch_command->Start();
    }
    if (m_operator_joystick->GetPOV(0) == 180) {
    	m_elevator_floor_command->Start();
    }
}


//==========================================================================
// Claw Operation

void Elevator::OpenClaw() {
	m_claw_solenoid->Set(true);
}

void Elevator::CloseClaw() {
	m_claw_solenoid->Set(false);
}

bool Elevator::IsClawOpen() {
	return m_claw_solenoid->Get();
}

void Elevator::RunClawRollerGrab(int timeout_s) {
	SetClawRollerVelocity(RobotConfiguration::kClawGrabVelocity);
	m_roller_state = RollerState::Grabbing;
	StartRollerTimeout(timeout_s);
}

void Elevator::RunClawRollerEject(int timeout_s,bool slow) {
	if (slow) {
		SetClawRollerVelocity(RobotConfiguration::kClawEjectVelocity/2);
	} else {
		SetClawRollerVelocity(RobotConfiguration::kClawEjectVelocity);
	}
	m_roller_state = RollerState::Ejecting;
	m_cube_loaded = false;
	StartRollerTimeout(timeout_s);
}

void Elevator::StopClawRollers() {
	SetClawRollerVelocity(0.0);
	m_roller_state = RollerState::Stopped;
	StopRollerTimeouts();
}

bool Elevator::IsCubeLoaded() {
	return m_cube_loaded;
}

void Elevator::SetCubeLoaded() {
	m_cube_loaded = true;
}


//==========================================================================
// Arm Operation

void Elevator::LiftArm() {
	// Default solenoid position is up
	std::cout << "Elevator::LiftArm()\n";
	m_arm_solenoid->Set(false);
}

void Elevator::DropArm() {
	std::cout << "Elevator::DropArm()\n";
	m_arm_solenoid->Set(true);
}

bool Elevator::IsArmUp(){
	return !m_arm_solenoid->Get();
}


//==========================================================================
// Elevator Operation

void Elevator::MoveLiftToPosition(LiftPosition position) {

	//Top limit switch: 36300

	// Record the lift position to drive to. The lift will be moved to this position
	// when DriveLiftToPosition() is called from the periodic update.
	m_lift_position = position;

	// If movement to intermediate is specified turn off any drive to the lift.
	if (position == LiftPosition::kIntermediate) {
		DoLiftDrive(0.0);
	}
}

Elevator::LiftPosition Elevator::GetLiftPosition() {
	// Get the height of the lift in inches
	double height_inch = GetLiftHeightInch();

	// Test each of the preset positions to see if the lift is above the position,
	// but within the tolerance. A one sided tolerance is used so that the lift is
	// always as least as high as it should be.
	// NOTE: The floor position is an exception as it is OK be to below it as this
	//       can only be noise in the encoder (it flickers to -1).
	if (height_inch <= kLiftFloorPositionInch + kLiftPositionToleranceInch) {
		return LiftPosition::kFloor;
	}

	if (height_inch >= kLiftArmLiftPositionInch &&
		height_inch <= kLiftArmLiftPositionInch + kLiftPositionToleranceInch) {
		return LiftPosition::kArmLift;
	}

	if (height_inch >= kLiftSwitchPositionInch &&
		height_inch <= kLiftSwitchPositionInch + kLiftPositionToleranceInch) {
		return LiftPosition::kSwitch;
	}

	if (height_inch >= kLiftScalePositionInch &&
		height_inch <= kLiftScalePositionInch + kLiftPositionToleranceInch) {
		return LiftPosition::kScale;
	}

	// If the lift is not at any preset position, return the 'intermediate' value
	return LiftPosition::kIntermediate;
}

double Elevator::GetLiftHeightInch() {
	int encoder_position = m_lift_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx);
	return encoder_position / kLiftEncodeCountsPerInch;
}

double Elevator::GetLiftPositionHeightInch(LiftPosition position) {
	switch (position) {
		case LiftPosition::kFloor:   return kLiftFloorPositionInch;
		case LiftPosition::kArmLift: return kLiftArmLiftPositionInch;
		case LiftPosition::kSwitch:  return kLiftSwitchPositionInch;
		case LiftPosition::kScale:   return kLiftScalePositionInch;
		default:
		case LiftPosition::kIntermediate:
			wpi_assert(false);
			return kLiftFloorPositionInch;
	}
}

double Elevator::GetLiftRaiseMovementScale() {
	// The scaling factor applies a full height and is linearly introduced from one third of full
	// height.
	double height_inch = GetLiftHeightInch();
	double start_height_inch = kLiftScalePositionInch/3.0;

	if (m_high_lift_slow_enabled) {

		if (height_inch < start_height_inch) {
			// Below the start height the drive base goes at full speed
			return 1.0;
		}  else {
			// Calculate a ratio that starts a 0 when the scaling begins and increase to 1
			// at the full scale height.
			double ratio = (height_inch - start_height_inch)/(kLiftScalePositionInch - start_height_inch);

			// Perform a linear interpolation between 1.0 at the start height and the
			// full 'movement scale' at full lift height.
			double scale_result = 1.0*(1.0 - ratio) + RobotConfiguration::kLiftRaisedMovementScale*ratio;
			if (scale_result >= RobotConfiguration::kLiftRaisedMovementScale)
				return scale_result;
			else
				return RobotConfiguration::kLiftRaisedMovementScale;
		}
	} else {
		return 1.0;
	}
}


//==========================================================================
// Claw Rollers Implementation

void Elevator::SetClawRollerVelocity(double speed) {
	// Roller motor run in opposite directions because they mirror each other
	m_left_claw_speed_controller->Set(ControlMode::PercentOutput, -speed);
	m_right_claw_speed_controller->Set(ControlMode::PercentOutput, speed);
}

void Elevator::StartRollerTimeout(int timeout_s) {
	// Record the timeout and start the timeout timer
	m_roller_timeout_s = timeout_s;
	m_roller_timeout_timer.Reset();
	m_roller_timeout_timer.Start();
}

void Elevator::StopRollerTimeouts() {
	// Clear the timeout value and the high current flag, and stop the timers
	m_roller_timeout_s = -1;
	m_roller_current_timer.Stop();
	m_roller_current_limit_exceeded = false;
	m_roller_current_timer.Stop();
}

void Elevator::CheckRollerCurrent() {
	// Do not apply the current limit unless we are grabbing the cube
	if (m_roller_state != RollerState::Grabbing) return;

	if (m_roller_current_limit_exceeded) {
		// If the current limit has been exceeded then check if enough time has passed to
		// stop the rollers and record that a cube is loaded.
		if (m_roller_current_timer.Get() >= RobotConfiguration::kClawMotorHighCurrentTimeS) {
			std::cout << "Roller stopped due to current exceeded. Recording cube loaded\n";
			m_cube_loaded = true;
			StopClawRollers();
		}
	} else {
		// No high current has been detected so far so get the current now (use the average
		// current from both motors) and test if it is high enough to indicate a cube.
		double roller_current = (m_left_claw_speed_controller->GetOutputCurrent() +
							     m_right_claw_speed_controller->GetOutputCurrent()) / 2.0;
		if (roller_current >= RobotConfiguration::kClawMotorGrabCurrentA) {
			// The current is high so set a flag and start a timer that will actually
			// stop the motors after a short delay.
			std::cout << "Roller current exceeded " << roller_current << "A\n";
			m_roller_current_limit_exceeded = true;
			m_roller_current_timer.Reset();
			m_roller_current_timer.Start();
		}
	}
}

void Elevator::CheckRollerTimeout() {
	// If the roller is moving and the timeout is set, see if the timeout has been
	// exceeded.
	if (m_roller_state != RollerState::Stopped && m_roller_timeout_s != -1) {
		if (m_roller_timeout_timer.Get() > m_roller_timeout_s) {
			StopClawRollers();
		}
	}
}


//==========================================================================
// Lift Implementation

void Elevator::DoLiftDrive(double percentage_velocity) {
	// If the lift is being driven at a particular speed then clear any
	// position that it is meant to drive to. It can only do velocity
	// or position, not both.
	m_lift_position = LiftPosition::kIntermediate;

	// Set the lift to the given drive.
	m_lift_speed_controller->Set(ControlMode::PercentOutput, percentage_velocity);
}

void Elevator::DriveLiftToPosition() {
	// If the lift is not driving to a position just return
	if (m_lift_position == LiftPosition::kIntermediate) return;

	// If the lift is driving to the floor and has got there turn off the
	// drive.
	if (m_lift_position == LiftPosition::kFloor && GetLiftPosition() == LiftPosition::kFloor) {
		m_lift_position = LiftPosition::kIntermediate;
		DoLiftDrive(0.0);
		return;
	}

	// Otherwise continue driving to the position. Note that for raised positions the
	// lift must be continually driven to the desired position, otherwise it will
	// slowly fall, even though the Talon motor controller is set to 'brake' mode.

	// Calculate a target position in inches that is in the middle of the target tolerance band.
	double target_position_inch = GetLiftPositionHeightInch(m_lift_position);
	target_position_inch += kLiftPositionToleranceInch / 2.0;

	// Calculate a target position in encoder counts
	double target_position_encoder = target_position_inch * kLiftEncodeCountsPerInch;

	// Use close loop position control to drive the lift to the desired location
	m_lift_speed_controller->SelectProfileSlot(kRunProfileSlotIdx, kPidDefaultIdx);
	m_lift_speed_controller->Set(ControlMode::Position, target_position_encoder);
}
