//==============================================================================
// RobotConfiguration.h
//==============================================================================

#ifndef SRC_ROBOTCONFIGURATION_H_
#define SRC_ROBOTCONFIGURATION_H_


namespace RobotConfiguration {
//==============================================================================
// CAN Bus Configuration

// Drive train Talon SRX controller CAN ids.
// NOTE: These must be checked carefully, because if two motors on the same side are
// driven against each other bad things will happen.
const int kLeftMasterTalonId  = 14;
const int kLeftSlaveTalonId   = 13;
const int kLeftSlave2TalonId  = 12;

const int kRightMasterTalonId = 3;
const int kRightSlaveTalonId  = 2;
const int kRightSlave2TalonId = 1;

// Elevator Talon SRX controller CAN ids.
const int kGripperLeftTalonId  = 10;
const int kGripperRightTalonId = 11;

const int kLiftTalonId = 0;

// CAN id of the CTRE Pigeon IMU (direct CAN bus connection)
const int kPigeonImuId = 0;


//==============================================================================
// Drive Base Configuration

// Continuous current limit for drive base motors in amps
const int kDriveMotorContinuousCurrentLimitA = 20;

// Peak current limit for drive base motors in amps
const int kDriveMotorPeakCurrentLimitA = 30;

// Peak current duration for drive base motors in milliseconds
const int kDriveMotorPeakCurrentDurationMs = 500;

// Drive motor ramp rate in second from neutral to full, for drive control (used for close loop)
const double kDriveMotorRampRateDriverS = 0.8;

// Drive motor ramp rate in second from neutral to full, for autonomous (used for open loop)
const double kDriveMotorRampRateAutonomousS = 0.0;

// Nominal (i.e. minimum) output for drive base motors on a scale of [0, 1]
const double kDriveMotorNominalOutput = 0.1;

// Maximum drive base speed in RPM. Measured by the encoders, that is on the wheel axis.
const double kDriveBaseMaxRpm = 450.0;

// Drive base wheel diameter in inches
//const double kWheelDiameterInch = 6.0;
//const double kWheelDiameterInch = 7.29;
const double kWheelDiameterInch = 6.25;

// Drive base close loop velocity control PID parameters
const double kDriveBasePidF = 0.3;
const double kDriveBasePidP = 0.05;
const double kDriveBasePidI = 0.0;
const double kDriveBasePidD = 0.0;


//==============================================================================
// Claw Configuration

// Peak current limit for claw intake motors in amps
const int kClawMotorPeakCurrentLimitA = 35;

// Continuous current limit for claw intake motors in amps
const int kClawMotorContinuousCurrentLimitA = 30;

// Peak current limit duration for claw intake motors in milliseconds
const int kClawMotorPeakCurrentDurationMs = 500;

// Claw roller current that indicates a cube has been grabbed, in amps
const int kClawMotorGrabCurrentA = 30;

// Time that the claw motors run for, in seconds, after high current has been
// detected so that cube is fully seated in the claw.
const double kClawMotorHighCurrentTimeS = 0.3;

// Time that the claw motors run for, in seconds, after manual claw button on
// the joystick is released.
const double kClawMotorReleaseButtonTimeS = 5;

// Percentage claw motor velocity when grabbing the cube (must be positive 0.0 to 1.0)
// This is maximum to grab cubes quickly, firmly and reliably.
const double kClawGrabVelocity = +1.0;

// Percentage claw motor velocity when ejecting the cube (must be negative 0.0 to -1.0)
// This is less than maximum so cubes can be placed carefully.
const double kClawEjectVelocity = -0.65;


//==============================================================================
// Lift Configuration

// Peak current limit for the lift motor in amps
const int kLiftMotorPeakCurrentLimit = 20;

// Continuous current limit for the lift motor in amps
const int kLiftMotorContinuousCurrentLimit = 15;

// Peak current limit duration for the lift motor in milliseconds
const int kLiftMotorPeakCurrentDurationMs = 500;


//==============================================================================
// Pneumatics Configuration

// CAN id of the pneumatics control module
const int kPneumaticsControlModuleId = 0;

// Id of the solenoid that controls raising and lowering the elevator arm
const int kPneumaticsArmSolenoidId = 0;

// Id of the solenoid that controls opening and closing the elevator claw
const int kPneumaticsClawSolenoidId = 1;


//==============================================================================
// Joystick Configuration

// The axes of different XBox joystick controls. These values are passed to GetRawAxis().
const int kJoystickLeftXAxis = 0;
const int kJoystickLeftYAxis = 1;
const int kJoystickLeftTriggerAxis = 2;
const int kJoystickRightTriggerAxis = 3;
const int kJoystickRightXAxis = 4;
const int kJoystickRightYAxis = 5;

// The values of different XBox joystick buttons. These values are passed to GetRawButton().
const int kJoystickAButton = 1;
const int kJoystickBButton = 2;
const int kJoystickXButton = 3;
const int kJoystickYButton = 4;
const int kJoystickLTrigButton = 5;
const int kJoystickRTrigButton = 6;
const int kJoystickLStartButton = 7;
const int kJoystickRStartButton = 8;
const int kJoystickLDialUp = 9;
const int kJoystickRDialUp = 10;

// The joystick port numbers used for the driver and operator
const int kJoystickPortDriver = 0;
const int kJoystickPortOperator = 1;

//==============================================================================
// Cheezy Drive Parameters

// Deadzone for the joystick. Joystick values close to zero that this limit are
// treated as zero. This is necessary because the joystick does not recentre exactly.
const double kJoystickDeadzone = 0.05;

// Exponential power to apply to the joystick movement and rotation values. These
// mean that there is finer control over slow speed and fine turning.
const double kMovePower = 2.0;
const double kRotatePower = 1.0;

// Scaling factor applied to rotation when driving with the joystick. This makes rotation
// slower and more controllable.
const double kRotateJoystickScale = 0.45;


// Scaling factor applied to rotate and move when driving with the evelator raised. This
// scaling factor applies a full height and is linearly introduced from one third of full
// height.
const double kLiftRaisedMovementScale = 0.5;



}

#endif /* SRC_ROBOTCONFIGURATION_H_ */
