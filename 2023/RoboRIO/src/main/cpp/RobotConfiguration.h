//==============================================================================
// RobotConfiguration.h
//==============================================================================

#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <numbers>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

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

// Drive train Talon FX controller CAN ids.
const int kLeftFrontDriveFalconId  = 13;
const int kLeftFrontSteerFalconId  = 12;
const int kRightFrontDriveFalconId  = 2;
const int kRightFrontSteerFalconId  = 3;
const int kLeftBackDriveFalconId  = 15;
const int kLeftBackSteerFalconId  = 14;
const int kRightBackDriveFalconId  = 0;
const int kRightBackSteerFalconId  = 1;

// CAN ids of the CTRE Talon FX controllers for Falcon 500 motors
const int kArmTalonId = 8;
const int kPivotTalonId = 6; 

// CAN ids of the CTRE Talon SRX controllers for manipulator motors
const int kIntakeTalonId = 4; 
const int kWinchTalonId = 5;
const int kWristTalonId = 9;

// CAN ids of the CTRE CANcoder (direct CAN bus connection)
const int kLeftFrontCANcoderId = 0;
const int kRightFrontCANcoderId = 1;
const int kLeftBackCANcoderId = 2;
const int kRightBackCANcoderId = 3;

// CAN id of the CTRE Pigeon IMU (direct CAN bus connection)
const int kPigeonImuId = 0;

// CAN id of the pneumatics control module
const int kPneumaticsControlModuleId = 0;


//==============================================================================
// Pneumatics Configuration


//==============================================================================
// Digital I/O Configuration


//==============================================================================
// Analog I/O Configuration


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
const units::inch_t kWheelDiameterInch = 4.0_in;

// Drive base gear box ratio (Swerve Dirve Specialties L1)
const double kDriveBaseGearRatio = 8.14;

// Drive base steering gear ratio (Swerve Dirve Specialties)
const double kDriveBaseSteerGearRatio = 150.0/7.0;

// Track width of the drivebase from left to right across the robot
const units::meter_t kDriveBaseWheelbaseWidth = 24_in;

// Track length of the drivebase from front to back along the robot
const units::meter_t kDriveBaseWheelbaseLength = 28_in;


const units::meters_per_second_t kMaxSpeed = 3.0_mps;  // 3 meters per second
const units::radians_per_second_t kMaxAngularSpeed { 2 * std::numbers::pi };  // 1 rotation per second

const units::radians_per_second_t kModuleMaxAngularVelocity =  std::numbers::pi * 2_rad_per_s;
const auto kModuleMaxAngularAcceleration = kModuleMaxAngularVelocity / 0.15_s;  // radians per second^2


// Default value for overall slow down factor. Should always be whatever the driver is happy
// with during the season (probably 1.0). After that it should be reduced to a safe speed for
// demos.
const double kDefaultDrivebaseSlowDownFactor = 0.9;


//==============================================================================
// Drive Base Motor Configuration

// Continuous current limit for drive base motors in amps
const int kDriveMotorContinuousCurrentLimitA = 20;

// Peak current limit for drive base motors in amps
const int kDriveMotorPeakCurrentLimitA = 30;

// Peak current duration for drive base motors in milliseconds  
const int kDriveMotorPeakCurrentDurationMs = 500;

// Drive motor ramp rate in second from neutral to full
//const double kDriveMotorRampRateS = 0.8;
const double kDriveMotorRampRateS = 0.4;

// Nominal (i.e. minimum) output for drive base motors on a scale of [0, 1]
//const double kDriveMotorNominalOutput = 0.05;
const double kDriveMotorNominalOutput = 0.05;


// Maximum drive base speed in RPM. Measured by the encoders, that is on the wheel axis.
const double kDriveBaseMaxRpm = 5000.0;

// Drive base close loop velocity control PID parameters
const double kDriveBasePidF = 0.047;
const double kDriveBasePidP = 0.1077;
const double kDriveBasePidI = 0.0;
const double kDriveBasePidD = 0.0;

// Drive base steering PID parameters
const double kSteerPidF = 0.044;
const double kSteerPidP = 0.0;
const double kSteerPidI = 0.0;
const double kSteerPidD = 0.0;


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

const units::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;

const double kPXController = 0.5;
const double kPYController = 0.5;
const double kPThetaController = 0.5;

const frc::TrapezoidProfile<units::radians>::Constraints 
    kThetaControllerConstraints{kMaxAngularSpeed, kModuleMaxAngularAcceleration};
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
//  VISION CONFIGURATION
//==============================================================================

// Transformation from the robot centre to the camera
const frc::Transform3d kRobotToCam =
                    frc::Transform3d(frc::Translation3d(0.34_m, 0_m, 12_in),
                    frc::Rotation3d(0_rad, 0_rad, 0_rad));


//==============================================================================
//  MANIPULATOR CONFIGURATION
//==============================================================================

//==============================================================================
// Intake Configuration

// Peak current limit for the intake motor in amps
const int kIntakeMotorPeakCurrentLimit = 25;

// Continuous current limit for the intake motor in amps
const int kIntakeMotorContinuousCurrentLimit = 18;

// Peak current limit duration for the intake motor in milliseconds
const int kIntakeMotorPeakCurrentDurationMs = 500;

// Peak current limit for the winch motor in amps
const int kWinchMotorPeakCurrentLimit = 10;

// Continuous current limit for the winch motor in amps
const int kWinchMotorContinuousCurrentLimit = 8;

// Peak current limit duration for the winch motor in milliseconds
const int kWinchMotorPeakCurrentDurationMs = 500;


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
const units::inch_t kArmMaximumExtensionInch = 31.0_in; 


//==============================================================================
// Pivot Configuration

// Pivot current limts
const int kPivotMotorPeakCurrentLimit = 15;
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
const units::degree_t kPivotMaximumForwardDegrees = 0.0_deg;
const units::degree_t kPivotMaximumReverseDegrees = -88_deg;

// Pivot gear ratio. The pivot is slowed down by this ratio compared to the encoder.
// 30:1 gearbox, 60t to 12t sprocket ratio
const double kPivotGearRatio = 30.0 * 60.0/12.0;

//==============================================================================
// Wrist Configuration

// Peak current limit for the wrist motor in amps
const int kWristMotorPeakCurrentLimit = 3;

// Continuous current limit for the wrist motor in amps
const int kWristMotorContinuousCurrentLimit = 2;

// Peak current limit duration for the wrist motor in milliseconds
const int kWristMotorPeakCurrentDurationMs = 500;

// Wrist motor ramp rate in second from neutral to full
const double kWristMotorRampRateDriverS = 0.8;

// Exponential power to apply to the joystick movement of the pivot. These
// mean that there is finer control over slow speed and fine turning.
const double kWristJoystickPower = 2.0;

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
const int kJoystickPovUp        =   0;
const int kJoystickPovUpRight   =  45;
const int kJoystickPovRight     =  90;
const int kJoystickPovDownRight = 135;
const int kJoystickPovDown      = 180;
const int kJoystickPovDownLeft  = 225;
const int kJoystickPovLeft      = 270;
const int kJoystickPovUpLeft    = 315;


//==============================================================================
// Joystick Ports

// The joystick port numbers used for the driver and operator
const int kJoystickPortDriver = 0;
const int kJoystickPortOperator = 1;


//==============================================================================
// Joystick Parameters

// Deadzone for the joystick. Joystick values close to zero that this limit are
// treated as zero. This is necessary because the joystick does not recentre exactly.
const double kJoystickDeadzone = 0.11;

}
