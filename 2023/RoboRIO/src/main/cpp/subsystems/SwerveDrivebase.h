//==============================================================================
// SwerveDrivebase.h
//==============================================================================

#pragma once

#include <frc2/command/SubsystemBase.h>
//#include <frc2/command/Command.h>

#include "SwerveModule.h"
#include "../commands/AutonomousCommand.h"
#include "../pathfollower/SwerveFollowerModuleState.h"
#include "../pathfollower/SwerveTrajectory.h"
#include "../RobotConfiguration.h"

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <ctre/Phoenix.h>

#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimVisionSystem.h>
#include <photonlib/SimVisionTarget.h>

#include <memory>
#include <string>
#include <vector>

namespace frc
{
class Joystick;
class Talon;
class Ultrasonic;
class ShuffleboardTab;
}
class HapticController;
class Manipulator;
class SwerveDrivebaseShuffleboard;

namespace pathplanner { class SwerveAutoBuilder; }


namespace RC = RobotConfiguration;
// DriveBase controls everything to do with the robot drive base, including:
//  - 4 swerve moduels
//  - Pigeon IMU
//  - Drive joystick object
class SwerveDrivebase : public frc2::SubsystemBase {
public:
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
    const frc::Pose2d& GetPose() const;
    
    // Get whether manual driving is currently field relative 
    bool GetFieldRelative() const;

    // Get const access to the swerve modules for monitoring their state
    wpi::array<const SwerveModule*,4> GetModules() const;

    // Get the current speed and rotational velocity of the drivebase
    frc::ChassisSpeeds GetChassisSpeeds();

    // Get the kinematics object for controlling the drivebase
    frc::SwerveDriveKinematics<4>& GetKinematics() { return *m_swerve_drive_kinematics; }

    // Get the Pigeon IMU heading in degrees. This angle 'winds up' and so is not restricted to any particular range
    units::degree_t GetPigeonHeading();

    // Get the robot pitch from the Pigeon IMU
    units::degree_t GetPitch() const;

    // Get the robot roll from the Pigeon IMU
    units::degree_t GetRoll() const;

    // PhotonLib pose estimation from the last update
    std::optional<photonlib::EstimatedRobotPose> GetLatestPhotonPose() const { return m_latest_photon_pose; }

    // PhotonLib result from the last update, only valid of there is a pose
    photonlib::PhotonPipelineResult GetLatestPhotonResult() const { return m_latest_photon_result; }


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

    // TODOVISION
    frc::SwerveDrivePoseEstimator<4>* GetEstimator() const;

    frc2::CommandPtr CreateTrajectoryCommand(const frc::Trajectory& trajectory);

    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

    void SetFollowerModuleStates(wpi::array<SwerveFollowerModuleState, 4> desiredStates);


private:
    //==========================================================================
    // Private Nested Types

    // Struct to hold the steering calibration values for the 4 swerve modules
    struct SteeringCalibration {
        units::degree_t m_left_front_angle;
        units::degree_t m_right_front_angle;
        units::degree_t m_left_back_angle;
        units::degree_t m_right_back_angle;
    };

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
    // Steering Calibration

    // File used for steering calibration.
    const char* CALIBRATION_FILENAME = "/home/lvuser/SwerveCalibration.txt";

    // Save the steering calibration from the current module positions
    void SaveSteeringCalibrationFromCurrentPosition();

    // Save the give steering calibrations to the file
    //
    // calibration - Calibration values to save
    void SaveSteeringCalibration(const SteeringCalibration& calibration);

    // Load the steering calibrations from the file
    //
    // calibration - Calibration values to set to the loaded values
    //
    // Returns whether loading is successful
    bool LoadSteeringCalibration(SteeringCalibration& calibration);

    // Log the give steering calibrations to the console output
    //
    // calibration - Calibration values to log
    void LogSteeringCalibration(const SteeringCalibration& calibration);
    

    //==========================================================================
    // Member Variables

    SwerveDrivebaseShuffleboard* m_shuffleboard = nullptr;  // Shuffleboard controller for this drivebase

    Manipulator* m_manipulator = nullptr;                   // Reference to the manipulator (so we can slow down when extended)

    SwerveModule* m_left_front_swerve_module = nullptr;     // Left front swerve module
    SwerveModule* m_right_front_swerve_module = nullptr;    // Right front swerve module
    SwerveModule* m_left_back_swerve_module = nullptr;      // Left back swerve module
    SwerveModule* m_right_back_swerve_module = nullptr;     // Right back swerve module

    frc::XboxController* m_controller = nullptr;            // XBox controller
    WPI_PigeonIMU* m_pigen_imu = nullptr;                   // Pigeon IMU (inertial measurement unit)
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

    photonlib::PhotonPoseEstimator* m_photon_estimator = nullptr;
    frc::SwerveDrivePoseEstimator<4>* m_swerve_estimator = nullptr;
    photonlib::SimVisionSystem* m_sim_vision = nullptr;
    std::vector<frc::AprilTag> m_april_tags;
    std::optional<photonlib::EstimatedRobotPose> m_latest_photon_pose;
                                                            // PhotonLib pose estimation from the last update
    photonlib::PhotonPipelineResult m_latest_photon_result; // PhotonLib result from the last update, only valid of there is a pose
};