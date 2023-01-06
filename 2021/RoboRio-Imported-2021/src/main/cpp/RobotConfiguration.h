//==============================================================================
// RobotConfiguration.h
//==============================================================================

#ifndef SRC_ROBOTCONFIGURATION_H_
#define SRC_ROBOTCONFIGURATION_H_

//#define COMPETITION_ROBOT
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
//  - Shooter Configuration
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
const int kLeftMasterTalonId  = 15;
const int kLeftSlaveTalonId   = 14;

const int kRightMasterTalonId = 1;
const int kRightSlaveTalonId  = 2;

const int kShooterMasterTalonId  = 5;
const int kShooterSlaveTalonId  = 6;
const int kIndexerTalonId = 10;
const int kIntakeTalonId = 12;

const int kWinchTalonId = 11;
    

// CAN id of the CTRE Pigeon IMU (direct CAN bus connection)
const int kPigeonImuId = 0;

// CAN id of the pneumatics control module
const int kPneumaticsControlModuleId = 0;


//==============================================================================
// Pneumatics Configuration

// Id of the forward side of the double solenoid that controls the kicker
const int kPneumaticsKickerForwardSolenoidId = 0;

// Id of the reverse side of the double solenoid that controls the kicker
const int kPneumaticsKickerReverseSolenoidId = 1;

// Id of the solenoid that controls the intake
const int kPneumaticsIntakeSolenoidId = 2;

// Id of the solenoid that controls the winch brake
const int kPneumaticsWinchBrakeSolenoidId = 3;


//==============================================================================
// Digital I/O Configuration


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

// Number of counts for a full revolution of a Talog FX encoder
const int kTalonFXEnocderCounts = 2048;

// The time base for Talon SRX velocity measurements in seconds. Velocities are
// measured in encoder counts per 100ms
const double kTalonTimeBaseS = 0.100;


//==============================================================================
//  DRIVEBASE CONFIGURATION
//==============================================================================

//==============================================================================
// Drive Base Physical Parameters

// Drive base wheel diameter in inches
const double kWheelDiameterInch = 6.0;

// Drive base gear box ratio
const double kDriveBaseGearRatio = ((60.0/10.0)*(42.0/24.0));


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
const double kDriveMotorNominalOutput = 0.05;

// Maximum drive base speed in RPM. Measured by the encoders, that is on the wheel axis.
const double kDriveBaseMaxRpm = 5000.0;

// Drive base close loop velocity control PID parameters
const double kDriveBasePidF = 0.047;
const double kDriveBasePidP = 0.1077;
const double kDriveBasePidI = 0.0;
const double kDriveBasePidD = 0.0;


//==============================================================================
// Drive Base Autonomous Configuration

// Drive base equation velocity constant [output/m/s]
const double kDriveBaseVelocityConstant = 0.208747534;

// Drive base equation velocity offset [output]
const double kDriveBaseVelocityOffset = 0.0;  // Measured as -0.004778574

// Drive base equation acceleration constant [output/m/s2]
const double kDriveBaseAccelerationConstant = 0.035884186;

// Drive base track width [m]
const double kDriveBaseTraceWidth = 0.64;


//==============================================================================
// Cheezy Drive Parameters

// Exponential power to apply to the joystick movement and rotation values. These
// mean that there is finer control over slow speed and fine turning.
const double kMovePower = 2.0;
const double kRotatePower = 1.5;

// Scaling factor applied to rotation when driving with the joystick. This makes rotation
// slower and more controllable.
const double kRotateJoystickScale = 0.4;

// Scaling factor applied to rotate and move when driving with the climbing arm raised
const double kClimbingArmRaisedMovementScale = 0.5;

// Minimum move value to ever user for manual control
const double kMoveMinimum = 0.04;

// Minimum absolute rotate value to ever user for manual control
const double kRotateMinimum = 0.03;

//==============================================================================
//  MANIPULATOR CONFIGURATION
//==============================================================================

//==============================================================================
// Shooter Configuration

// Peak current limit for the shooter motors in amps
const int kShooterMotorPeakCurrentLimit = 50;

// Continuous current limit for the shooter motors in amps
const int kShooterMotorContinuousCurrentLimit = 40;

// Peak current limit duration for the shooter motors in milliseconds
const int kShooterMotorPeakCurrentDurationMs = 500;

// Gear ratio, baby
const double kShooterMotorGearRatio = 1.5;


//==============================================================================
// Winch Configuration

// Peak current limit for the winch motor in amps
const int kWinchMotorPeakCurrentLimit = 50;

// Continuous current limit for the winch motor in amps
const int kWinchMotorContinuousCurrentLimit = 40;

// Peak current limit duration for the winch motor in milliseconds
const int kWinchMotorPeakCurrentDurationMs = 500;

// Diameter of the winch drum in inches
const double kWinchDiameterInch = 2.0;

// Maximum extension of the winch hook cable in inches
const double kWinchMaximumExtensionInch = 26
;

// Maximum speed of the winch as a fraction of full speed
const double kWinchSpeedFraction = 0.8;


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
const double kJoystickDeadzone = 0.07;

}

#endif /* SRC_ROBOTCONFIGURATION_H_ */
