//==============================================================================
// RobotConfiguration.h
//==============================================================================

#ifndef SRC_ROBOTCONFIGURATION_H_
#define SRC_ROBOTCONFIGURATION_H_

#define COMPETITION_ROBOT
//#define PRACTICE_ROBOT

namespace RobotConfiguration {

//==============================================================================
// Overview
//
// This file contains all the configuration data needed for the robot. A central
// location is used to prevent duplication and to 
//
// The configuration parameters are quite complex and so they are organised under
// headings and sub-headings as follows. Please make sure new parameters are
// inserted in the corrent location, creating new headings as necessary.
//
//  SHARED RESOURCES CONFIGURATION
//  - CAN Bus Configuration
//  - Pneumatics Configuration
//  - Digital I/O Configuration
//  - Analog I/O Configuration
//  - Common Talon SRX Configuration
//
//  DRIVEBASE CONFIGURATION
//  - Drive Base Physical Parameters
//  - Drive Base Motor Configuration
//  - Cheezy Drive Parameters
//
//  MANIPULATOR CONFIGURATION
//  - Zucc Configuration
//  - Wrist Configuration
//  - Arm Configuration
//  - Pivot Configuration
//  - Intake Configuration
//
//  JOYSTICK CONFIGURATION
//  - Joystick Axes and Buttons
//  - Joystick Ports
//  - Joystick Parameters

//==============================================================================
//  SHARED RESOURCES CONFIGURATION
//==============================================================================

//==============================================================================
// CAN Bus Configuration

// CAN bus ids must be unique for each class of device so they are all defined here
// to avoid conflicts

// Drive train Talon SRX controller CAN ids.
// NOTE: These must be checked carefully, because if two motors on the same side are
// driven against each other bad things will happen.
const int kLeftMasterTalonId  = 15;   // 14;
const int kLeftSlaveTalonId   = 14;   // 13;
const int kLeftSlave2TalonId  = 13;   // 12;

const int kRightMasterTalonId = 0;    // 3;
const int kRightSlaveTalonId  = 1;    // 2;
const int kRightSlave2TalonId = 2;    // 1;

// Manipulator Talon SRX controller CAN ids.
const int kIntakeRetractTalonId  = 5; // 10;
const int kIntakeRollerTalonId = 3;   // 11;
const int kWristTalonId = 9;
const int kArmTalonId = 8;
const int kPivotTalonId = 12;         // 22;

// CAN id of the CTRE Pigeon IMU (direct CAN bus connection)
const int kPigeonImuId = 0;

// CAN id of the pneumatics control module
const int kPneumaticsControlModuleId = 0;


//==============================================================================
// Pneumatics Configuration

// Id of the solenoid that controls the continuous power output for Zucc
const int kPneumaticsConstantPowerSolenoidId = 0;

// Id of the  solenoid that controls the direction the Zucc
const int kPneumaticsZuccDirectionSolenoidId = 1;


//==============================================================================
// Digital I/O Configuration

// Id of the digital output that controls turning on Zucc vaccumn
const int kDigitalOutZuccVacuumId = 0; //7;

// Id of the digital output that controls turning on Zucc release
const int kDigitalOutZuccReleaseId = 1; // 6;

// Id of the digital input that controls Zucc suction indicator
const int kDigitalInZuccSuctionIndicatorId = 9; // 4;

// Id of the digital output that triggers the Zucc ultrasonic sensor
const int kUltrasonicTriggerPort = 2;

// Id of the digital input that listens for an echo Zucc ultrasonic sensor
const int kUltrasonicEchoPort = 3;

// Id of the digital output that triggers the foreward driving ultrasonic sensor
const int kForwardUltrasonicTriggerPort = 2;

// Id of the digital input that listens for the foreward driving ultrasonic sensor
const int kForwardUltrasonicEchoPort = 1;

// Id of the digital output that triggers the backward driving ultrasonic sensor
const int kBackwardUltrasonicTriggerPort = 2;

// Id of the digital input that listens for the backward driving ultrasonic sensor
const int kBackwardUltrasonicEchoPort = 1;


//==============================================================================
// Analog I/O Configuration

// Id of the analog input for the air pressure sensor
const int kAnalogInPressureSensorId = 0;


//==============================================================================
// Common Talon SRX Configuration
//
// For configuration that can be used by all Talon SRX speed controllers

// Use PID index 0, meaning primary closed loop
const int kTalonPidIdx = 0;

// Talon configuration timeout in ms.  If nonzero, function will wait for config 
// success and report an error if it times out. If zero, no blocking or checking
// is performed. 
const int kTalonTimeoutMs = 10;

// PID profile slot id set up in the code and used when running the robot
const int kTalonRunProfileSlotIdx = 0;

// PID profile slot id set up via the web interface and to use when tuning PID parameters
const int kTalonTuneProfileSlotIdx = 1;

// Number of counts for a full revolution of a CTRE magnetic encoder
const int kCtreEnocderCounts = 4096;

// The time base for Talon SRX velocity measurements in seconds. Velocities are
// measured in encoder counts per 100ms
const double kTalonTimeBaseS = 0.100;


//==============================================================================
//  DRIVEBASE CONFIGURATION
//==============================================================================

//==============================================================================
// Drive Base Physical Parameters

// Drive base wheel diameter in inches
const double kWheelDiameterInch = 8.0;


//==============================================================================
// Drive Base Motor Configuration

// Continuous current limit for drive base motors in amps
const int kDriveMotorContinuousCurrentLimitA = 20;

// Peak current limit for drive base motors in amps
const int kDriveMotorPeakCurrentLimitA = 30;

// Peak current duration for drive base motors in milliseconds
const int kDriveMotorPeakCurrentDurationMs = 500;

// Drive motor ramp rate in second from neutral to full
const double kDriveMotorRampRateS = 0.8;

// Nominal (i.e. minimum) output for drive base motors on a scale of [0, 1]
const double kDriveMotorNominalOutput = 0.1;

// Maximum drive base speed in RPM. Measured by the encoders, that is on the wheel axis.
const double kDriveBaseMaxRpm = 450.0;

// Drive base close loop velocity control PID parameters
const double kDriveBasePidF = 0.3;
const double kDriveBasePidP = 0.05;
const double kDriveBasePidI = 0.0;
const double kDriveBasePidD = 0.0;


//==============================================================================
// Cheezy Drive Parameters

// Exponential power to apply to the joystick movement and rotation values. These
// mean that there is finer control over slow speed and fine turning.
const double kMovePower = 2.0;
const double kRotatePower = 1.0;

// Scaling factor applied to rotation when driving with the joystick. This makes rotation
// slower and more controllable.
const double kRotateJoystickScale = 0.8;

// Scaling factor applied to rotate and move when driving with the evelator raised. This
// scaling factor applies a full height and is linearly introduced from one third of full
// height.
const double kLiftRaisedMovementScale = 0.5;


//==============================================================================
//  MANIPULATOR CONFIGURATION
//==============================================================================

//==============================================================================
// Zucc Configuration


//==============================================================================
// Wrist Configuration

// Peak current limit for the wrist motor in amps
const int kWristMotorPeakCurrentLimit = 2;

// Continuous current limit for the wrist motor in amps
const int kWristMotorContinuousCurrentLimit = 1;

// Peak current limit duration for the wrist motor in milliseconds
const int kWristMotorPeakCurrentDurationMs = 500;

// Wrist motor ramp rate in second from neutral to full
const double kWristMotorRampRateDriverS = 0.8;

// Exponential power to apply to the joystick movement of the pivot. These
// mean that there is finer control over slow speed and fine turning.
const double kWristJoystickPower = 2.0;


//==============================================================================
// Arm Configuration

// Peak current limit for the arm motor in amps
const int kArmMotorPeakCurrentLimit = 20;

// Continuous current limit for the arm motor in amps
const int kArmMotorContinuousCurrentLimit = 15;

// Peak current limit duration for the arm motor in milliseconds
const int kArmMotorPeakCurrentDurationMs = 500;

// Open loop ramp rate is long to give the operator control of the arm
const double kArmMotorOpenLoopRampRateS = 0.5;

// Close loop ramp rate is short so the PID operates properly
const double kArmMotorClosedLoopRampRateS = 0.05;

// Maximum extension of the arm in inches. This is the maximum extension required in the
// game. The maximum physical extension defined by the stops in the arm is slightly longer.
const double kArmMaximumExtensionInch = 44.0;  // TODO should be 44", but need to check that is safe

// Maximum allowed extension of the arm when horizontal in the forward and backwards
// positions. These are obtained from measurement. The backward value is negative
// because there the arm extends more than 15" when horizontal. The negative value
// means that no extension is allowed until the arm is point up somewhat.
const double kForwardMaximumExtensionInch = 3.5;
const double kBackwardMaximumExtensionInch = -10.0;

// Base length of the arm from pivot to the outedge of the zucc, when fully retracted
const double kArmLengthInch = 38.0;

//==============================================================================
// Pivot Configuration

// Pivot current limts
const int kPivotMotorPeakCurrentLimit = 12;
const int kPivotMotorContinuousCurrentLimit = 10;
const int kPivotMotorPeakCurrentDurationMs = 500;

// Open loop ramp rate is long to give the operator control of the pivot
const double kPivotMotorOpenLoopRampRateS = 0.8;

// Close loop ramp rate is short so the PID operates properly
const double kPivotMotorClosedLoopRampRateS = 0.05;

// Exponential power to apply to the joystick movement of the pivot. These
// mean that there is finer control over slow speed and fine turning.
const double kPivotJoystickPower = 2.0;

// Maximum drive applied with the joystick
const double kPivotJoystickMax = 0.5;

// Maximum rotation of the pivot in degrees. This is set to a range of values for testing
// the pivot safely. For the purposes of testing we assume a starting position of straight up
// and zero the count there. 
const double kPivotMaximumForwardDegrees = 86.0;
const double kPivotMaximumReverseDegrees = -108.0;

// Pivot gear ratio. The pivot is slowed down by this ratio compared to the encoder.
#ifdef COMPETITION_ROBOT
const double kPivotGearRatio = 54.0/15.0;
#endif
#ifdef PRACTICE_ROBOT
const double kPivotGearRatio = 60.0/15.0; 
#endif


//==============================================================================
// Intake Configuration

// Peak current limit for the intake motors in amps
const int kIntakeRetractMotorPeakCurrentLimit = 15;
const int kIntakeRollerMotorPeakCurrentLimit = 15;

// Continuous current limit for the intake motors in amps
const int kIntakeRetractMotorContinuousCurrentLimit = 10;
const int kIntakeRollerMotorContinuousCurrentLimit = 10;

// Peak current limit duration for the intake motors in milliseconds
const int kIntakeRetractMotorPeakCurrentDurationMs = 500;
const int kIntakeRollerMotorPeakCurrentDurationMs = 500;

// Intake roller motor current that indicates that a ball as been grabbed, in Amps
const double kIntakeRollerMMotorGrabCurrentA = 3.0;

// Time to run the intake roller for after high current is detected, in seconds
const double kIntakeRollerMotorHighCurrentTimeS = 0.5;
// Exponential power to apply to the joystick movement of the intake. These
// mean that there is finer control over slow speed and fine turning.
const double kIntakeJoystickPower = 2.0;

// Maximum drive applied with the joystick
const double kIntakeJoystickMax = 0.5;

// Intake retract gear ratio. The intake retract is slowed down by this ratio compared to the encoder.
#ifdef COMPETITION_ROBOT
const double kIntakeRetractRatio = 1.0; // TODO is it 16/18??
#endif
#ifdef PRACTICE_ROBOT
const double kIntakeRetractRatio = 1.0;
#endif


//==============================================================================
//  JOYSTICK CONFIGURATION
//==============================================================================


//==============================================================================
// Joystick Axes and Buttons

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

// The angle returned for POV directions
const int kJoystickPovUp    = 0;
const int kJoystickPovRight = 90;
const int kJoystickPovDown  = 180;
const int kJoystickPovLeft  = 270; // TODO - Confirm this


//==============================================================================
// Joystick Ports

// The joystick port numbers used for the driver and operator
const int kJoystickPortDriver = 0;
const int kJoystickPortOperator = 1;


//==============================================================================
// Joystick Parameters

// Deadzone for the joystick. Joystick values close to zero that this limit are
// treated as zero. This is necessary because the joystick does not recentre exactly.
const double kJoystickDeadzone = 0.05;

}

#endif /* SRC_ROBOTCONFIGURATION_H_ */
