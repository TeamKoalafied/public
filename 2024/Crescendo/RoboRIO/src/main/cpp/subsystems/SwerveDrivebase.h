//==============================================================================
// SwerveDrivebase.h
//==============================================================================

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"
#include "../commands/AutonomousCommand.h"
#include "../RobotConfiguration.h"
#include "../util/PovFilter.h"

#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace frc
{
class Joystick;
class Talon;
class Ultrasonic;
class ShuffleboardTab;
}
class DriveShuffleboard;
class HapticController;
class Manipulator;
class SwerveDrivebaseShuffleboard;
class Vision;


namespace RC = RobotConfiguration;
// DriveBase controls everything to do with the robot drive base, including:
//  - 4 swerve moduels
//  - Pigeon IMU
//  - Drive joystick object
class SwerveDrivebase : public frc2::SubsystemBase {
public:
    //==========================================================================
    // Public Nested Types

    // State of the drivebase
    enum class State {
        Target,     // Turning to a 'target' (either speaker or lob shot)
        Idle,       // Normal driving
    };

    // State of the current targeting operation
    enum class TargetState {
        SpeakerTargeting,     // Turning towards the speaker
        LobTargeting,         // Turning towards the amp corner for a lob shot
        OnTarget,             // On target for shooting or lobbing
        Failed,               // Targeting has failed
    };
    
    
    //==========================================================================
    // Construction

    // Constructor
    SwerveDrivebase(Manipulator* manipulator);  

    // Destructor
    virtual ~SwerveDrivebase();


    //==========================================================================
    // frc::Subsystem Function Overrides
    //virtual void InitDefaultCommand() override;   
    virtual void Periodic() override;
    virtual void SimulationPeriodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Set the reference to the drive shuffleboard for getting slow down factor
    void SetDriveShuffleboard(DriveShuffleboard* drive_shuffleboard) {
        m_drive_shuffleboard = drive_shuffleboard;
    }

    // Setup the drive base for robot operation, with encoders, slave drive etc.
    void Setup();

    // Shutdown the drive base
    void Shutdown();

    // Initialise for teleop operation
    void TeleopInit();

    // Initialise for disabled operation
    void DisabledInit();

    // Initialise for simulation
    void SimulationInit();


    //==========================================================================
    // Drivebase State

    // Get the current robot pose from the odometry
    const frc::Pose2d GetPose() const;
    
    // Get whether manual driving is currently field relative 
    bool GetFieldRelative() const;

    // Get const access to the swerve modules for monitoring their state
    const wpi::array<const SwerveModule*,4>& GetModules() const { return *m_modules_array; }

    // Get the current speed and rotational velocity of the drivebase
    frc::ChassisSpeeds GetChassisSpeeds();

    // Get the kinematics object for controlling the drivebase
    frc::SwerveDriveKinematics<4>& GetKinematics() const { return *m_swerve_drive_kinematics; }

    // Get the odometry (position of robot with any vision updating)
    frc::SwerveDriveOdometry<4>* GetOdometry() const { return m_swerve_drive_odometry; }

    // Get the Pigeon IMU rotation
    frc::Rotation2d GetPigeonRotation() const;

    // Get the Pigeon IMU heading in degrees. This angle 'winds up' and so is not restricted to any particular range
    units::degree_t GetPigeonHeading() const;

    // Get the robot pitch from the Pigeon IMU
    units::degree_t GetPitch() const;

    // Get the robot roll from the Pigeon IMU
    units::degree_t GetRoll() const;

    units::inch_t GetDistanceToTarget() const;

    // Get the vision for the robot
    const Vision& GetVision() const { return *m_vision; }

    // Robot relative chassis speeds from the last call to Drive()
    const frc::ChassisSpeeds& GetDriveRobotSpeed() const { return m_drive_robot_speed; }

    // Whether the robot is pointing at the target when in shooting state
    bool GetPointingAtTarget() const { return m_pointing_at_target; }


    //==========================================================================
    // Operations

    // Drive with a given velocity and rotation
    //
    // x_speed - X direction speed
    // y_speed - Y direction speed
    // rotation - angular speed (anticlockwise is +ve)
    // field_relative - whether the speed is relative to the field, rather than the robot
    void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed,
               units::radians_per_second_t rotation, bool field_relative);

    // Drive with a given velocity and rotation, specified by a ChassisSpeeds object
    //
    // chassis_speeds - speed to drive at
    // field_relative - whether the speed is relative to the field, rather than the robot
    void Drive(const frc::ChassisSpeeds& chassis_speeds, bool field_relative);


    // Stop the drivebase
    void Stop() { Drive(0_mps, 0_mps, 0_rad_per_s, false); }

    // Reset the pose of the robot in the odometry
    //
    // pose - Pose to set the robot to
    void ResetPose(const frc::Pose2d& pose);

    // Turn the wheels without moving to be able to drive with a given velocity
    //
    // x_speed - X direction speed
    // y_speed - Y direction speed
    // field_relative - whether the speed is relative to the field, rather than the robot
    void AlignWheelsToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative);

    // Check if the wheels are facing appropriately to be able to drive with a given velocity
    //
    // x_speed - X direction speed
    // y_speed - Y direction speed
    // field_relative - whether the speed is relative to the field, rather than the robot
    //
    // Whether the wheels are aligned, within a reasonable tolerance
    bool AreWheelsAlignedToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative);

    // Do slow driving with a given velocity
    //
    // x_speed - X direction speed
    // y_speed - Y direction speed
    // field_relative - whether the speed is relative to the field, rather than the robot
    void SlowDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative);

    // Turn the wheels to 45 degrees to lock the robot position
    void LockWheels();

    // Set the brake/coast mode for the drive motors
    //
    // brake - Whether to set brake mode (otherwise coast)
    void SetBrakeMode(bool brake);


    //==========================================================================
    // Special Development Operations

    // Do open loop driving for drivebase characterisation
    //
    // x_speed_fraction - X direction fractional speed
    // y_speed_fraction - Y direction fractional speed
    // rotation - angular speed (anticlockwise is +ve)
    // field_relative - whether the speed is relative to the field, rather than the robot
    void CharacterisationDrive(double x_speed_fraction, double y_speed_fraction, units::radians_per_second_t rotation, bool field_relative);


private:
    //==========================================================================
    // State Control
    
    // Takes the requested input state and updates the internal state accordingly
    // state - requested state from the controller
    void DoState(State state);

    // Calls the setup function for a new state
    void EnterState();
    
    // Calls the periodic update function for the current state
    void UpdateState();

    // Calls the exit function for the current state
    void ExitState();

    // Performs setup for the targeting state
    void EnterTargetingState();

    // Updates the targeting state
    void UpdateTargetingState();

    // Exits the targeting state
    void ExitTargetingState();


    //==========================================================================
    // Joystick Control

    // Do joystick control of the robot
    void DoJoystickControl();

    // Do slow driving with a given velocity, turning wheels before moving for precise control
    //
    // x_speed - X direction speed
    // y_speed - Y direction speed
    // field_relative - whether the speed is relative to the field, rather than the robot
    void ManualDriveSlow(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative);

    // Do joystick control of the robot for motor tuning
    void DoTuningJoystickControl();

    // Do joystick control of the robot for motor tuning of a single swerve module
    //
    // swerve_module - the swerve module that is being tuned
    void DoTuningModuleJoystickControl(SwerveModule* swerve_module);


    //==========================================================================
    // Toe In Calculation

    // Calculate the swerve module states to drive in a given drirection with 'toe-in' angling off
    // the serve modules to holefully given more traction
    //
    // x_speed - Speed in the x direction, relative to the robot
    // y_speed - Speed in the y direction, relative to the robot
    // toe_in_angle - Toe-in angle to use
    //
    // Returns the required module states
    wpi::array<frc::SwerveModuleState, 4> CalculateToeInSwerveModuleStates(units::meters_per_second_t x_speed,
        units::meters_per_second_t y_speed, units::degree_t toe_in_angle);


    //==========================================================================
    // Steering Calibration

    // Save the steering calibration from the current module positions
    void SaveSteeringCalibrationFromCurrentPosition();
    

    //==========================================================================
    // Member Variables

    SwerveDrivebaseShuffleboard* m_shuffleboard = nullptr;  // Shuffleboard controller for this drivebase

    Manipulator* m_manipulator = nullptr;                   // Reference to the manipulator (so we can slow down when extended)
    DriveShuffleboard* m_drive_shuffleboard = nullptr;      // Reference to the drive shuffleboard for getting slow down factor

    SwerveModule* m_left_front_swerve_module = nullptr;     // Left front swerve module
    SwerveModule* m_right_front_swerve_module = nullptr;    // Right front swerve module
    SwerveModule* m_left_back_swerve_module = nullptr;      // Left back swerve module
    SwerveModule* m_right_back_swerve_module = nullptr;     // Right back swerve module
    wpi::array<const SwerveModule*,4>* m_modules_array = nullptr;
                                                            // Swerve modules grouped in an array 

    frc::XboxController* m_controller = nullptr;            // XBox controller
    PovFilter m_pov_filter;                                 // Filter for POV so that it only returns the four major directions
    ctre::phoenix6::hardware::Pigeon2* m_pigeon_imu = nullptr;
                                                            // Pigeon IMU (inertial measurement unit)
    HapticController* m_haptic_controller = nullptr;        // Controller for haptic feedback through the controller

    frc::SwerveDriveKinematics<4>* m_swerve_drive_kinematics = nullptr;
                                                            // Kinematics object for controlling the drivebase
    frc::SwerveDriveOdometry<4>* m_swerve_drive_odometry = nullptr;
                                                            // Odometry object for tracking the drivebase position on the field

    frc::SlewRateLimiter<units::scalar> m_xspeed_limiter{3 / 1_s};
                                                            // Ramp limiter for robot x direction speed
    frc::SlewRateLimiter<units::scalar> m_yspeed_limiter{3 / 1_s};
                                                            // Ramp limiter for robot y direction speed
    frc::SlewRateLimiter<units::scalar> m_rotation_limiter{3 / 1_s};
                                                            // Ramp limiter for robot rotation speed

    bool m_field_relative = true;                           // Flag indicating whether manual driving is currently field relative
    bool m_manual_drive_slow_aligned = false;               // Flag to indicate that aligning the wheels for drive slow is complete
    frc::Timer m_manual_drive_slow_aligned_timer;           // Timer how long aligning the wheels for drive slow takes
    int m_manual_distance_index = 0;                        // Index of manual shooting distances

    Vision* m_vision;                                       // Controller for all the vision stuff
    State m_state = State::Idle;                            // State of the drivebase
    TargetState m_target_state;                             // State of the current targeting operation
    bool m_pointing_at_target;                              // Whether the robot is pointing at the target when in shooting state
    std::optional<frc2::CommandPtr> m_align_to_amp_command; // Command being run to align the drivebase to the amp

    frc::ChassisSpeeds m_drive_robot_speed;                 // Robot relative chassis speeds from the last call to Drive()
};