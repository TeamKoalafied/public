//==============================================================================
// DriveBase.cpp
//==============================================================================


#include "DriveBase.h"

#include "math.h"
#include "frc/WPILib.h"

#include "RobotConfiguration.h"
#include "KoalafiedUtilities.h"
#include "Commands/DriveWithJoystick.h"
#include "Commands/Test/TestCharacteriseDriveBase.h"

#include <frc/Talon.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "TalonConfig.h"

namespace RC = RobotConfiguration;

using namespace frc;

//==============================================================================
// Construction

DriveBase::DriveBase() :
    TSingleton<DriveBase>(this),
    frc::Subsystem("DriveBase") {

    m_left_master_speed_controller = NULL;
    m_left_slave_speed_controller = NULL;
    m_left_slave2_speed_controller = NULL;
    m_right_master_speed_controller = NULL;
    m_right_slave_speed_controller = NULL;
    m_right_slave2_speed_controller = NULL;

    m_forward_ultrasonic = NULL;
    m_backward_ultrasonic = NULL;

    m_joystick = NULL;
    m_pigen_imu = NULL;

    m_driving_straight = false;
    m_straight_heading = 0.0;

    m_log_counter = 0;
}

DriveBase::~DriveBase() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void DriveBase::InitDefaultCommand() {
	// Unless the drive base is doing an autonomous command it is controlled by the joystick
    SetDefaultCommand(new DriveWithJoystick());
}

void DriveBase::Periodic() {

	// If either of the trigger buttons is down start the default command. This means that
	// the joystick can be used to cancel any autonomous command, in case something going wrong.
	if (m_joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton) ||
		m_joystick->GetRawButton(RobotConfiguration::kJoystickRTrigButton)) {
		if (!GetDefaultCommand()->IsRunning()) {
			GetDefaultCommand()->Start();
		}
	}

	// Record the heading from the pigeon in a circular buffer
	double heading = GetPigeonHeading();
	m_heading_buffer[m_heading_buffer_index++] = heading;
	if (m_heading_buffer_index > TOTAL_HEADINGS) m_heading_buffer_index = 0;

	// Record the velocity into a circular buffer
	double velocity = GetVelocityFeetPerSecond();
	double old_velocity = m_velocity_buffer[m_velocity_buffer_index];
	m_velocity_buffer[m_velocity_buffer_index++] = velocity;
	if (m_velocity_buffer_index > TOTAL_VELOCITIES) m_velocity_buffer_index = 0;

    bool display_dashboard = false;
    if (display_dashboard) {
        // Display the left and right motor powers
        SmartDashboard::PutNumber("LeftMotor", m_left_master_speed_controller->GetMotorOutputPercent());
        SmartDashboard::PutNumber("RightMotor", m_right_master_speed_controller->GetMotorOutputPercent());
        double distance_right = EncoderToInches(m_right_master_speed_controller->GetSelectedSensorPosition(0));

        SmartDashboard::PutNumber("LeftPos", GetDistanceInch());
        SmartDashboard::PutNumber("RightPos",distance_right);
        SmartDashboard::PutNumber("MVel", velocity);

        double acceleration = (velocity - old_velocity)/(TOTAL_VELOCITIES * 0.02);
        SmartDashboard::PutNumber("Acceleration", acceleration);

        SmartDashboard::PutBoolean("Drive Joystick", GetDefaultCommand()->IsRunning());

        double left_current = m_left_master_speed_controller->GetOutputCurrent() + 
                              m_left_slave_speed_controller->GetOutputCurrent() +
                              m_left_slave2_speed_controller->GetOutputCurrent();
        double right_current = m_right_master_speed_controller->GetOutputCurrent() + 
                               m_right_slave_speed_controller->GetOutputCurrent() +
                               m_right_slave2_speed_controller->GetOutputCurrent();
        SmartDashboard::PutNumber("Motor Left Current", left_current);
        SmartDashboard::PutNumber("Motor Right Current", right_current);

        //LogDebug();
    //	virtual double GetBusVoltage();
    //	virtual double GetMotorOutputPercent();
    //	virtual double GetMotorOutputVoltage();
    //	virtual double GetOutputCurrent();
    //	virtual double GetTemperature();

        //PigeonImu::GeneralStatus pigeon_status;
        //m_pigen_imu->GetGeneralStatus(pigeon_status);
        SmartDashboard::PutNumber("Heading", heading);
    }
}

//==============================================================================
// Setup and Shutdown

void DriveBase::Setup() {
    m_joystick = new frc::Joystick(0);
    m_pigen_imu = new PigeonIMU(RobotConfiguration::kPigeonImuId);
    m_pigen_imu->SetFusedHeading(0.0,kTalonTimeoutMs);

    m_heading_buffer_index = 0;
    for (int i = 0; i < TOTAL_HEADINGS; i++) m_heading_buffer[i] = 0;
    m_velocity_buffer_index = 0;
    for (int i = 0; i < TOTAL_VELOCITIES; i++) m_velocity_buffer[i] = 0;

    m_vision_light_relay = new frc::Relay(0);

    // Create controllers for each of the 4 drive talons
    m_left_master_speed_controller = new TalonSRX(RobotConfiguration::kLeftMasterTalonId);
    m_left_slave_speed_controller = new TalonSRX(RobotConfiguration::kLeftSlaveTalonId);
    m_left_slave2_speed_controller = new TalonSRX(RobotConfiguration::kLeftSlave2TalonId);
    m_right_master_speed_controller = new TalonSRX(RobotConfiguration::kRightMasterTalonId);
    m_right_slave_speed_controller = new TalonSRX(RobotConfiguration::kRightSlaveTalonId);
    m_right_slave2_speed_controller = new TalonSRX(RobotConfiguration::kRightSlave2TalonId);
//	virtual int GetFirmwareVersion();

    // Setup the slave Talons to follow the masters
    m_left_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kLeftMasterTalonId);
    m_left_slave2_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kLeftMasterTalonId);
    m_right_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kRightMasterTalonId);
    m_right_slave2_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kRightMasterTalonId);

    // Log whether the encoders are connected
//    printf("Left magnetic encode present: %s ",
//           (m_left_master_speed_controller->IsSensorPresent(TalonSRX::CtreMagEncoder_Absolute) ? "true" : "false"));
//    printf("Right magnetic encode present: %s ",
//           (m_right_master_speed_controller->IsSensorPresent(TalonSRX::CtreMagEncoder_Absolute) ? "true" : "false"));

    // Set the encoders to be the feedback devices for closed loop control on the master motors
    // TSSRM Section 7 (page 43)
    m_left_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, kPidDefaultIdx, kTalonTimeoutMs);
    m_left_master_speed_controller->SetSensorPhase(true); // Not reversed
    m_right_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, kPidDefaultIdx, kTalonTimeoutMs);
    m_right_master_speed_controller->SetSensorPhase(true); // Not reversed

    // Velocity measurement configuration
    // TSSRM Section 7.8 (page 50)
//    m_left_master_speed_controller->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period, , kTalonTimeoutMs);
//    m_left_master_speed_controller->ConfigVelocityMeasurementWindow(int windowSize, kTalonTimeoutMs);
//    m_right_master_speed_controller->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period, , kTalonTimeoutMs);
//    m_right_master_speed_controller->ConfigVelocityMeasurementWindow(int windowSize, kTalonTimeoutMs);
    masterConfigs MASTER_CONFIG;
    slaveConfigs SLAVE_CONFIG;
    // emil's configall testing zone
    m_left_master_speed_controller->ConfigAllSettings(MASTER_CONFIG._talon, kTalonTimeoutMs);
    m_right_master_speed_controller->ConfigAllSettings(MASTER_CONFIG._talon, kTalonTimeoutMs);
    m_left_slave_speed_controller->ConfigAllSettings(SLAVE_CONFIG._talon, kTalonTimeoutMs);
    m_right_slave_speed_controller->ConfigAllSettings(SLAVE_CONFIG._talon, kTalonTimeoutMs);
    m_left_slave2_speed_controller->ConfigAllSettings(SLAVE_CONFIG._talon, kTalonTimeoutMs);
    m_right_slave2_speed_controller->ConfigAllSettings(SLAVE_CONFIG._talon, kTalonTimeoutMs);

    // Set the peak and nominal voltage outputs for the master motors. This is for closed loop only.
    // The peak outputs are the maximum, but the nominal (same in both directions) is tuned to be
    // about the minimum value that will overcome the drive train friction.
    // TSSRM Section 10.5 (page 66)

    // Set the ramp rate for open and close loop modes. TODO Is this necessary for slaves?
    // TSSRM Section 6 (page 41)
    

    // Voltage compensation TODO Is this required?
    // TSSRM Section 9.2 (page 60)
//  virtual ctre::phoenix::ErrorCode ConfigVoltageCompSaturation(double voltage, int timeoutMs);
//	virtual ctre::phoenix::ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);
//	virtual void EnableVoltageCompensation(bool enable);

    // Set the continuous and peak current limits, for all motors. The is for all control modes (i.e. open/closed loop)
    // TSSRM Section 9.3 (page 62)
    m_left_master_speed_controller->EnableCurrentLimit(true);
    m_left_slave_speed_controller->EnableCurrentLimit(true);
    m_left_slave2_speed_controller->EnableCurrentLimit(true);
    m_right_master_speed_controller->EnableCurrentLimit(true);
    m_right_slave_speed_controller->EnableCurrentLimit(true);
    m_right_slave2_speed_controller->EnableCurrentLimit(true);

    // Set the PID controller parameters for the closed loop control of the master
    // motors. Use the profile slot for running the robot.

    // config for the configall is automatically set to 0, can't dynamically adjust (afaik), so keep in mind when changing kRunProfileSlotIdx
    m_left_master_speed_controller->SelectProfileSlot(kRunProfileSlotIdx, kPidDefaultIdx);

    m_right_master_speed_controller->SelectProfileSlot(kRunProfileSlotIdx, kPidDefaultIdx);

    m_left_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, kTalonTimeoutMs);
    m_left_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_, 20, kTalonTimeoutMs);
    m_right_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, kTalonTimeoutMs);
    m_right_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_, 20, kTalonTimeoutMs);

//	virtual ctre::phoenix::ErrorCode Config_IntegralZone(int slotIdx, int izone,
//			int timeoutMs);
//	virtual ctre::phoenix::ErrorCode ConfigAllowableClosedloopError(int slotIdx,
//			int allowableCloseLoopError, int timeoutMs);
//	virtual ctre::phoenix::ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, double iaccum,
//			int timeoutMs);


//	m_forward_ultrasonic = new frc::Ultrasonic(RC::kForwardUltrasonicTriggerPort, RC::kForwardUltrasonicEchoPort);
//	m_forward_ultrasonic->SetAutomaticMode(true);
//	m_backward_ultrasonic = new frc::Ultrasonic(RC::kBackwardUltrasonicTriggerPort, RC::kBackwardUltrasonicEchoPort);
//	m_backward_ultrasonic->SetAutomaticMode(true);

	ResetDistance();
}

void DriveBase::ClearState() {
	SetBrakeMode(false);
	ResetPigeonHeading();
}

void DriveBase::Shutdown() {
    // Delete the joystick and pigeon, and clear the pointers
    delete m_joystick;
    m_joystick = NULL;
    delete m_pigen_imu;
    m_pigen_imu = NULL;

    // Delete the all the speed controllers (it owns then and will delete then for us).
    delete m_left_master_speed_controller;
    m_left_master_speed_controller = NULL;
    delete m_left_slave_speed_controller;
    m_left_slave_speed_controller = NULL;
    delete m_left_slave2_speed_controller;
    m_left_slave2_speed_controller = NULL;
    delete m_right_master_speed_controller;
    m_right_master_speed_controller = NULL;
    delete m_right_slave_speed_controller;
    m_right_slave_speed_controller = NULL;
    delete m_right_slave2_speed_controller;
    m_right_slave2_speed_controller = NULL;
}

//==============================================================================
// Operation

void DriveBase::ResetJoystickState() {
	// Clear the joystick state, otherwise press and release events that
	// have already occurred my be incorrectly reported.
	KoalafiedUtilities::ClearJoystickButtonState(m_joystick);
}

void DriveBase::DoCheezyDrive() {
//	DoTuningDrive();
//	return;



    // Get the movement and rotation values from the joystick, including any speed
    // limiting and response curve shaping.
    double move = 0.0;
    double rotate = 0.0;
    GetMovementFromJoystick(move, rotate);
    CalculateDriveStraightAdjustment(move, rotate);

    // Get the robot drive to do arcade driving with our rotate and move values
    ArcadeDrive(move, rotate);

//   TestCharacteriseDriveBase::DoJoystickControl(m_joystick);
}

void DriveBase::StartDrivingStraight(double heading) {
	m_straight_heading = heading;
	m_driving_straight = true;
}

void DriveBase::Drive(double velocity_feet_per_second, double rotate) {
	// Convert the velocity into a proportional speed value
	double velocity_inch_per_second = velocity_feet_per_second * 12.0;
	double wheel_circumference_inch = RobotConfiguration::kWheelDiameterInch * 3.1415;
	double velocity_revolutions_per_second = velocity_inch_per_second / wheel_circumference_inch;
	double velocity_rpm = velocity_revolutions_per_second * 60.0;
	double velocity_proportional = velocity_rpm / RobotConfiguration::kDriveBaseMaxRpm;

	// Clip the velocity to the allowed range
	if (velocity_proportional > 1.0) velocity_proportional = 1.0;
	if (velocity_proportional < -1.0) velocity_proportional = -1.0;

	CalculateDriveStraightAdjustment(velocity_proportional, rotate);

	// Get the robot drive to do arcade driving with our rotate and move values
	ArcadeDrive(velocity_proportional, rotate);
}

void DriveBase::DriveDistance(double distance_inch, double velocity_feet_per_second) {
	// TODO: Consider removing Talon Motion Magic driving as it did not really work.

	double velocity_inch_per_second = velocity_feet_per_second * 12.0;
	double wheel_circumference_inch = RobotConfiguration::kWheelDiameterInch * 3.1415;
	double velocity_revolutions_per_second = velocity_inch_per_second / wheel_circumference_inch;
	double velocity_rpm = velocity_revolutions_per_second * 60.0;
	double velocity_proportional = velocity_rpm / RobotConfiguration::kDriveBaseMaxRpm;

	if (velocity_proportional > 1.0) velocity_proportional = 1.0;
	if (velocity_proportional < -1.0) velocity_proportional = -1.0;

	double motor_velocity_native = VelocityRmpToNative(velocity_proportional * RobotConfiguration::kDriveBaseMaxRpm);
	motor_velocity_native = fabs(motor_velocity_native);

	m_left_master_speed_controller->ConfigMotionCruiseVelocity(motor_velocity_native, kTalonTimeoutMs);
	m_right_master_speed_controller->ConfigMotionCruiseVelocity(motor_velocity_native, kTalonTimeoutMs);
	m_left_master_speed_controller->ConfigMotionAcceleration(2*motor_velocity_native, kTalonTimeoutMs);
	m_right_master_speed_controller->ConfigMotionAcceleration(2*motor_velocity_native, kTalonTimeoutMs);

	double encoder_target = InchesToEncoder(distance_inch);
	m_left_master_speed_controller->Set(ControlMode::MotionMagic, encoder_target);
	m_right_master_speed_controller->Set(ControlMode::MotionMagic, -encoder_target);

	std::cout << "DriveBase::DriveDistance() " << distance_inch << " , " << velocity_feet_per_second << "\n";
	std::cout << motor_velocity_native << ",  " << encoder_target << "\n";
}

void DriveBase::TankDriveOpenLoop(double left_output, double right_output) {
	// Just directly set the open loop outputs.
	m_left_master_speed_controller->Set(ControlMode::PercentOutput, left_output);
	m_right_master_speed_controller->Set(ControlMode::PercentOutput, -right_output);
}

void DriveBase::Stop() {
	// Set the drive to zero movement and rotation
	Drive(0.0, 0.0);

	// Turn on brake mode
	SetBrakeMode(true);
}

void DriveBase::SetBrakeMode(bool brake) {
	NeutralMode neutral_mode = brake ? NeutralMode::Brake : NeutralMode::Coast;

	m_left_master_speed_controller->SetNeutralMode(neutral_mode);
	m_left_slave_speed_controller->SetNeutralMode(neutral_mode);
	m_left_slave2_speed_controller->SetNeutralMode(neutral_mode);
	m_right_master_speed_controller->SetNeutralMode(neutral_mode);
	m_right_slave_speed_controller->SetNeutralMode(neutral_mode);
	m_right_slave2_speed_controller->SetNeutralMode(neutral_mode);
}


//==========================================================================
// Distance

void DriveBase::ResetDistance() {
    // If there is no Speed Controller just log an error and do nothing
    if (m_left_master_speed_controller == NULL || m_right_master_speed_controller == NULL) {
    	std::cout << "Error: SpeedController is null in DriveBase::ResetDistance()\n";
    	return;
    }

    // Display the distance before we reset, as this is helpful when debugging autonomous driving code
    std::cout << "Distance Reset: previous distance " << GetDistanceInch()<< "\n";

    // Reset the left and right encoders
    if (m_left_master_speed_controller->SetSelectedSensorPosition(0, kPidDefaultIdx, kTalonTimeoutMs) != 0) {
    	std::cout << "Error: Left Controller SetSelectedSensorPosition() failed in DriveBase::ResetDistance()\n";
    }

    if (m_right_master_speed_controller->SetSelectedSensorPosition(0, kPidDefaultIdx, kTalonTimeoutMs) != 0) {
        std::cout << "Error: Right Controller SetSelectedSensorPosition() failed in DriveBase::ResetDistance()\n";
    }

    // When the enocoder are reset they do not read back zero immediately. So we wait until they do,
    // otherwise following code could read the wrong start distance value and do something bad. However
    // we limit the amount of waiting because sometimes we may not get a zero read back because the
    // robot is moving, and then the code would hang forever.
    int count = 0;
    while (EncoderToInches(m_left_master_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx)) != 0 &&
    	   count < 200) {
    	count++;
    }
    std::cout << count << " tries before Reset\n";
}

double DriveBase::GetDistanceInch() {
	// Get the distance as measured by the left and right motor controllers
    double distance_left_inch = EncoderToInches(m_left_master_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx));
    double distance_right_inch = -EncoderToInches(m_right_master_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx));

    // Need average now for DrivePath
    return (distance_left_inch + distance_right_inch)/2;
}

void DriveBase::GetEncoderDistances(int& left_encoder, int& right_encoder) {
	left_encoder = m_left_master_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx);
	right_encoder = m_right_master_speed_controller->GetSelectedSensorPosition(kPidDefaultIdx);
}


double DriveBase::GetVelocityFeetPerSecond()
{
    double left_speed_rpm = VelocityNativeToRmp(m_left_master_speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx));
    double right_speed_rpm = VelocityNativeToRmp(m_right_master_speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx));
    double velocity_rpm = (left_speed_rpm - right_speed_rpm)/2.0;

	double wheel_circumference_inch = RobotConfiguration::kWheelDiameterInch * 3.1415;

	return velocity_rpm * wheel_circumference_inch / (12 * 60);
}

double DriveBase::GetPigeonHeading() {
    // If there is no Pigeon IMU return an error
    if (m_pigen_imu == NULL) return kHeadingError;

    // If the Pigeon IMU is not ready return an error
    if (m_pigen_imu->GetState() != PigeonIMU::Ready) return kHeadingError;

    // The Pigeon has multiple methods for determining a heading.
    // For the moment we are using the 'fused' heading.
    return m_pigen_imu->GetFusedHeading();
}

double DriveBase::GetStablePigeonHeading() {
	// Get the minimum and maximum headings recorded in the heading buffer
	double min_heading = m_heading_buffer[0];
	double max_heading = m_heading_buffer[0];
	double total_heading = m_heading_buffer[0];
    for (int i = 1; i < TOTAL_HEADINGS; i++) {
    	double heading = m_heading_buffer[i];
    	if (heading == kHeadingError) return kHeadingError;
    	if (min_heading > heading) min_heading = heading;
    	if (max_heading < heading) max_heading = heading;
    	total_heading += heading;
    }

    // If the range of headings is less than a small limit then return the
    // average heading as a stable heading value. Otherwise return a heading
    // error, meaning that the heading is not stable.
    const double kStableHeadingDeltaDegrees = 3;
    if (fabs(min_heading - max_heading) <= kStableHeadingDeltaDegrees) {
    	return total_heading / (double)TOTAL_HEADINGS;
    } else {
    	return kHeadingError;
    }
}

int DriveBase::GetPigeonRawGyro(double gyro_xyz_dps[3]) {
    // If there is no Pigeon IMU return an error
    if (m_pigen_imu == NULL) return kGyroError;

    // If the Pigeon IMU is not ready return an error
    if (m_pigen_imu->GetState() != PigeonIMU::Ready) return kGyroError;

    // For the moment we are using the gyro to return rotate speed
    m_pigen_imu->GetRawGyro(gyro_xyz_dps);
    return kGyroTrue;
}

/*int sentient = 0;
  if (robot == sentient){
  robot = NULL;
  delete robot;
  sentience = 0;
  } else {
  for (int i; i>100; i++){
  	 sentience++;
  	 }
  }*/

void DriveBase::ResetPigeonHeading()
{
    // If there is no Pigeon IMU just log an error and do nothing
    if (m_pigen_imu == NULL) {
    	std::cout << "Error: Pigeon is null in DriveBase::ResetPigeonHeading()\n";
    	return;
    }

    // If the Pigeon IMU is not ready just log an error and do nothing
    if (m_pigen_imu->GetState() != PigeonIMU::Ready){
    	std::cout << "Error: Pigeon not ready in DriveBase::ResetPigeonHeading()\n";
    	return;
    }

    // Reset the fused heading to 0. Log if an error occurs.
    if (m_pigen_imu->SetFusedHeading(0.0, kTalonTimeoutMs) != 0) {
    	std::cout << "Error: Pigeon SetFusedHeading() failed in DriveBase::ResetPigeonHeading()\n";
    }

    std::cout << "Pigeon Heading Reset";
}


//==========================================================================
// Motor Performance

double DriveBase::GetMotorVoltage()
{
	return m_left_master_speed_controller->GetMotorOutputVoltage();
}


//==========================================================================
// Input Functions

void DriveBase::GetMovementFromJoystick(double& move, double& rotate) {
    // The rotate input comes directly from the x axis of the joystick.
    // Apply a 'deadzone' so that very small inputs all map to 0. This is
    // necessary because the joystick does not centre perfectly when released.
    double joystick_x = m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftXAxis);
    rotate = -joystick_x;
    if (fabs(rotate) < RobotConfiguration::kJoystickDeadzone) rotate = 0.0;

    // Forward movement is controlled by the right trigger and backwards
    // movement by the left trigger. We only register a trigger if the other
    // trigger is not also activated at the same time.
    move = 0.0;
    double left_trigger = m_joystick->GetRawAxis(RobotConfiguration::kJoystickLeftTriggerAxis);
    double right_trigger = m_joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis);
    if (left_trigger > 2*right_trigger) {
        move = -left_trigger;
    } else if (right_trigger > 2*left_trigger) {
        move = right_trigger;
    } else if (left_trigger > 0.0 && right_trigger > 0.0) {
		std::cout << "WARNING: Both driver triggers pressed\n";
    }

    // Apply a 'power' adjustment to the move and rotate values. This gives
    // more precision for small adjustments (this is like mouse pointer
    // acceleration).
    move   = KoalafiedUtilities::PowerAdjust(move, RobotConfiguration::kMovePower);
    rotate = KoalafiedUtilities::PowerAdjust(rotate, RobotConfiguration::kRotatePower);

    // Scale down joystick rotation to make it more controllable,
    // unless the Y button is being held down
    if (!m_joystick->GetRawButton(RobotConfiguration::kJoystickYButton)) {
    	rotate *= RobotConfiguration::kRotateJoystickScale;
    }

    // Display the joystick inputs and the move value we calculate from them
    // SmartDashboard::PutNumber("JoystickX", joystick_x);
    // SmartDashboard::PutNumber("LeftTrigger", left_trigger);
    // SmartDashboard::PutNumber("RightTrigger", right_trigger);
    // SmartDashboard::PutNumber("Rotate", rotate);
    // SmartDashboard::PutNumber("Move", move);
}


//==========================================================================
// Drive Functions


void DriveBase::ArcadeDrive(double move_value, double rotate_value) {
	// Scale the rotation and movement to slow the drive base down when the
	// lift is in a high position.
//	double lift_scale_factor = Elevator::GetInstance().GetLiftRaiseMovementScale();
//	move_value *= lift_scale_factor;
//	rotate_value *= lift_scale_factor;

	// local variables to hold the computed PWM values for the motors
	double left_motor_output;
	double right_motor_output;

	if (move_value > 0.0) {
		if (rotate_value > 0.0) {
			left_motor_output = move_value - rotate_value;
			right_motor_output = std::max(move_value, rotate_value);
		} else {
			left_motor_output = std::max(move_value, -rotate_value);
			right_motor_output = move_value + rotate_value;
		}
	} else {
		if (rotate_value > 0.0) {
			left_motor_output = -std::max(-move_value, rotate_value);
			right_motor_output = move_value + rotate_value;
		} else {
			left_motor_output = move_value - rotate_value;
			right_motor_output = -std::max(-move_value, -rotate_value);
		}
	}

	// Scale the output from percentage to RPM and then convert to native units
	double left_motor_velocity_native = VelocityRmpToNative(left_motor_output * RobotConfiguration::kDriveBaseMaxRpm);
	double right_motor_velocity_native = VelocityRmpToNative(right_motor_output * RobotConfiguration::kDriveBaseMaxRpm);

	// If either motor is moving switch off brake mode
	if (left_motor_velocity_native != 0.0 || right_motor_velocity_native != 0.0) {
		SetBrakeMode(false);
	}

	// Set the velocity of the left and right motors. Note that the right motor
	// velocity is negated as it faces in the opposite direction and so must rotate
	// in the opposite direction for the robot to move forwards.
	m_left_master_speed_controller->Set(ControlMode::Velocity, left_motor_velocity_native);
	m_right_master_speed_controller->Set(ControlMode::Velocity, -right_motor_velocity_native);
    
    // not sure why needed but fixed oscillation when stationary
    if ((left_motor_velocity_native == 0) && (right_motor_velocity_native == 0)) {
        m_left_master_speed_controller->Set(ControlMode::PercentOutput, 0.0);
	    m_right_master_speed_controller->Set(ControlMode::PercentOutput, 0.0);
    }
}


void DriveBase::CalculateDriveStraightAdjustment(double move, double& rotate) {
    // If the rotation value is zero then we want to drive straight
    bool driving_straight = rotate == 0.0;

	if (move == 0.0){
		m_driving_straight = false;
		return;
	}

    // When we first start wanting to drive straight, record the heading
    // that we want to track
    if (driving_straight && !m_driving_straight) {
        // We now want to drive straight, so record the heading that we want to track
    	// However we only want to record the heading to start driving straight if the
    	// current direction is stable. If we record the direction while still turning
    	// the 'drive straight' will end up correcting us to the wrong direction (this
    	// can be seen by driving straight, then making a small turn and driving straight
    	// again). Hence we use the 'stable' heading, which returns an error when the
    	// heading has not been stable for a minimum period of time.
        m_straight_heading = GetStablePigeonHeading();
        // If there was a problem with the Pigeon IMU then clear the
        // 'driving_straight' flag. This means that we won't attempt to start
        // driving straight until we have a valid heading to work with.
        if (m_straight_heading == kHeadingError) {
        	driving_straight = false;
        } else {
			SmartDashboard::PutNumber("StraightHeading", m_straight_heading);
			printf("Starting drive straight %d  straight heading %f\n", driving_straight, m_straight_heading);
        }
    }

    // Record the new driving straight mode
    m_driving_straight = driving_straight;

    double straight_gain = SmartDashboard::GetNumber("Straight Gain", 0.01);

    // If in driving straight mode calculate a new rotate value based on the
    // error in the heading
    if (m_driving_straight) {
        // Get the current heading. If there is a problem with the Pigeon IMU
        // then we don't try to do any adjustments.
        double current_heading = GetPigeonHeading();
        if (current_heading != kHeadingError) {
            double error = m_straight_heading - current_heading;
            while (error > 180.0) error = error - 360.0;
            while (error < -180.0) error = error + 360.0;

            // TODO: The gain for driving straight is a random guess needs to be tuned.
            //       A full PID controller would be much better.
//            const double STRAIGHT_GAIN = 0.001;
//            rotate = error * STRAIGHT_GAIN;
            rotate = error * straight_gain;
        }
    }
}


//==========================================================================
// Calculation Functions

double DriveBase::VelocityRmpToNative(double velocity_rpm) {
    return velocity_rpm * 4096.0 / (60.0 * 10.0);
}

double DriveBase::VelocityNativeToRmp(double velocity_native) {
    return velocity_native * 60.0 *10.0 / 4096.0;
}

double DriveBase::EncoderToInches(int encoder_count) {
	double revolutions = encoder_count / 4096.0;
	double wheel_circumference_inch = RobotConfiguration::kWheelDiameterInch * 3.1415;
	return wheel_circumference_inch * revolutions;
}

double DriveBase::InchesToEncoder(int distance_inches) {
	double wheel_circumference_inch = RobotConfiguration::kWheelDiameterInch * 3.1415;
	double revolutions = distance_inches / wheel_circumference_inch;
	return revolutions * 4096.0;
}


//==========================================================================
// Motor Tuning

void DriveBase::DoTuningDrive() {
    // Only log the values every so often, to avoid slowing things down
	const int kUpdatePeriod = 25;
    bool log_value = false;
    m_log_counter++;
    if (m_log_counter == kUpdatePeriod) {
        m_log_counter = 0;
        log_value = true;
    }

    // Do the tuning of the left and right sides
    DoTuningDriveSide(-RobotConfiguration::kJoystickLeftYAxis, m_left_master_speed_controller, "LEFT", log_value);
    DoTuningDriveSide(RobotConfiguration::kJoystickRightYAxis, m_right_master_speed_controller, "RIGHT", log_value);
}

void DriveBase::DoTuningDriveSide(int joystick_axis, TalonSRX* speed_controller, const char* name, bool log_value) {
    // Get the speed in RPM.
    double speed_native = speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx);
    double speed_rpm = VelocityNativeToRmp(speed_native);

    // Get the close loop error and convert to RPM
    double closed_loop_error_native = speed_controller->GetClosedLoopError(kPidDefaultIdx);
    double closed_loop_error_rpm = VelocityNativeToRmp(closed_loop_error_native);

    // Display the current motor speed and close loop error at all times
    SmartDashboard::PutNumber(std::string(name) + " Velocity RPM", speed_rpm);
    SmartDashboard::PutNumber(std::string(name) + " Error RPM", closed_loop_error_rpm);
    SmartDashboard::PutNumber(std::string(name) + " Sensor", speed_controller->GetSelectedSensorPosition(kPidDefaultIdx));


	// Get the movement value. If it is effectively zero just stop the motor, so that
	// we don't log information for a side that is not moving.
    double move = m_joystick->GetRawAxis(abs(joystick_axis));
    if (joystick_axis < 0) move = -move;
    if (fabs(move) < RobotConfiguration::kJoystickDeadzone) {
        speed_controller->Set(ControlMode::PercentOutput, 0.0);
        SmartDashboard::PutNumber(std::string(name) + " Target RPM", 0.0);
    	return;
    }

    // Calculate the target speed in RPM and display
	double target_velocity_rpm = move * RobotConfiguration::kDriveBaseMaxRpm;
	double target_velocity_native = VelocityRmpToNative(target_velocity_rpm);
    SmartDashboard::PutNumber(std::string(name) + " Target RPM", target_velocity_rpm);

    // If the right trigger is pressed at all, do closed loop testing
    if (m_joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis) > 0.0) {
        // Run in close loop and report the error margin

        // Do closed loop velocity control and set a desired speed from
        // our movement value.
        speed_controller->Set(ControlMode::Velocity, target_velocity_native);

        // Set the controller to use the gains from the second profile slot. These
        // values will be set via the RoboRIO web interface during tuning.
        speed_controller->SelectProfileSlot(kTuneProfileSlotIdx, kPidDefaultIdx);
        if (log_value) {
            printf("%s: F %f  P %f  I %f  D %f\n", name,
            		speed_controller->ConfigGetParameter(eProfileParamSlot_F, kTuneProfileSlotIdx, kTalonTimeoutMs),
					speed_controller->ConfigGetParameter(eProfileParamSlot_P, kTuneProfileSlotIdx, kTalonTimeoutMs),
                    speed_controller->ConfigGetParameter(eProfileParamSlot_I, kTuneProfileSlotIdx, kTalonTimeoutMs),
					speed_controller->ConfigGetParameter(eProfileParamSlot_D, kTuneProfileSlotIdx, kTalonTimeoutMs));
        }

        // Calculate the motor output voltage as a fraction
        double motor_output = speed_controller->GetMotorOutputVoltage()/
                              speed_controller->GetBusVoltage();

        // Get the speed in RPM.
        double speed_native = speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx);
        double speed_rpm = VelocityNativeToRmp(speed_native);

        // Get the close loop error and convert to RPM
        double closed_loop_error_native = speed_controller->GetClosedLoopError(kPidDefaultIdx);
        double closed_loop_error_rpm = VelocityNativeToRmp(closed_loop_error_native);

        // Output the values if required
        if (log_value) {
            printf("%s: Output %f  Speed %f (%f rpm) Target %f (%f rpm) Error %f (%f)\n",
            		name, motor_output, speed_native, speed_rpm, target_velocity_native, target_velocity_rpm,
					closed_loop_error_native, closed_loop_error_rpm);
        }
    } else {
        // Run in open loop and calculate a value for F

        // Set the motor voltage directly from our movement value
        speed_controller->Set(ControlMode::PercentOutput, move);

        // Calculate the motor output voltage as a fraction
        double motor_output = speed_controller->GetMotorOutputVoltage()/
                              speed_controller->GetBusVoltage();

        // Get the speed in RPM and convert to the native units of encode counts
        // per 100ms time period (see TSSRM page 88).
        double speed_native = speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx);
        double speed_rpm = VelocityNativeToRmp(speed_native);

        // Calculate a feed forward gain (F) for this speed
        double F = motor_output * 1023.0/speed_native;

        // Output the values if required
        if (log_value) {
            printf("%s: Output %f  Speed %f  F %f\n", name, motor_output, speed_rpm, F);
        }
    }
}
