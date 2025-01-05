//==============================================================================
// SwerveDrivebase.cpp
//==============================================================================

#include "SwerveDrivebase.h"

#include "Manipulator.h"
#include "SwerveDrivebaseShuffleboard.h"
#include "SwerveSteeringCalibration.h"
#include "Vision.h"

#include "../DriveShuffleboard.h"
#include "../RobotConfiguration.h"
#include "../commands/AlignToAmpCommand.h"
#include "../commands/TestTurnCommand.h"
#include "../util/HapticController.h"
#include "../util/KoalafiedUtilities.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc/DriverStation.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include <units/length.h>

#include <fstream>
#include <iostream>
#include <iomanip>

namespace RC = RobotConfiguration;

//==============================================================================
// Construction 

SwerveDrivebase::SwerveDrivebase(Manipulator* manipulator) {
    m_manipulator = manipulator;
}

// Destructor
SwerveDrivebase::~SwerveDrivebase() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void SwerveDrivebase::Periodic() {

    // Update the odometry from the pigeon values and swerve module state 
    const wpi::array<frc::SwerveModulePosition,4U> module_positions{
        m_left_front_swerve_module->GetPosition(),
        m_right_front_swerve_module->GetPosition(),
        m_left_back_swerve_module->GetPosition(),
        m_right_back_swerve_module->GetPosition()
    };
    m_swerve_drive_odometry->Update(m_pigeon_imu->GetRotation2d(), module_positions);

    // Update the vision simulation
    wpi::array<double, 3> std_devs = m_shuffleboard->GetStdDevs();
    m_vision->Update(std_devs);

    units::degree_t lf_scrub = m_left_front_swerve_module->GetDesiredState().angle.Degrees() - module_positions[0].angle.Degrees();
    m_left_front_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(lf_scrub));
    units::degree_t rf_scrub = m_right_front_swerve_module->GetDesiredState().angle.Degrees() - module_positions[1].angle.Degrees();
    m_right_front_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(rf_scrub));
    units::degree_t lb_scrub = m_left_back_swerve_module->GetDesiredState().angle.Degrees() - module_positions[2].angle.Degrees();
    m_left_back_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(lb_scrub));
    units::degree_t rb_scrub = m_right_back_swerve_module->GetDesiredState().angle.Degrees() - module_positions[3].angle.Degrees();
    m_right_back_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(rb_scrub));

    // Update the haptic controller and shuffleboard. Note shuffle board is updated last as it
    // displays state that we have just calculated.
    m_haptic_controller->Periodic();
    m_shuffleboard->UpdateShuffleboard();
}

void SwerveDrivebase::SimulationPeriodic() {

    units::second_t update_time = 0.02_s;

    // Update each of the swerve modules
    m_left_front_swerve_module->UpdateSimulation(update_time);
    m_right_front_swerve_module->UpdateSimulation(update_time);
    m_left_back_swerve_module->UpdateSimulation(update_time);
    m_right_back_swerve_module->UpdateSimulation(update_time);

    // Combine the state of the 4 swerve modules to get the overall velocity and rotation
    // of the robot. Update the Pigeon heading for the current angular velocity over the
    // simulation update time.
    ctre::phoenix6::sim::Pigeon2SimState& pigeon_sim = m_pigeon_imu->GetSimState();
    frc::ChassisSpeeds chassis_speeds = GetChassisSpeeds();
    units::degree_t angle_increment = chassis_speeds.omega * update_time;
    pigeon_sim.AddYaw(angle_increment);

    // Update the vision simulation
    m_vision->UpdateSimulation();
}


//==============================================================================
// Setup and Shutdown

void SwerveDrivebase::Setup() {
    // Create the shuffleboard for the drivebase
    m_shuffleboard = new SwerveDrivebaseShuffleboard(this, m_manipulator);
    
    // Setup the Xbox controller
    m_controller = new frc::XboxController(RC::kJoystickPortDriver);
    m_haptic_controller = new HapticController(m_controller); 

    // Setup the Pigeon IMU
    m_pigeon_imu = new ctre::phoenix6::hardware::Pigeon2(RC::kPigeonImuId);
    m_pigeon_imu->Reset();

    // Create the TalonFX controllers for the Falcons
    units::radian_t left_front_zero_point = 0_rad;
    units::radian_t right_front_zero_point = 0_rad;
    units::radian_t left_back_zero_point = 0_rad;
    units::radian_t right_back_zero_point = 0_rad;
    m_left_front_swerve_module = new SwerveModule("Left Front", RC::kLeftFrontDriveFalconId, RC::kLeftFrontSteerFalconId,
                                                  RC::kLeftFrontCANcoderId, left_front_zero_point);
    m_right_front_swerve_module = new SwerveModule("Right Front", RC::kRightFrontDriveFalconId, RC::kRightFrontSteerFalconId,
                                                  RC::kRightFrontCANcoderId, right_front_zero_point);
    m_left_back_swerve_module = new SwerveModule("Left Back", RC::kLeftBackDriveFalconId, RC::kLeftBackSteerFalconId,
                                                  RC::kLeftBackCANcoderId, left_back_zero_point);
    m_right_back_swerve_module = new SwerveModule("Right Back", RC::kRightBackDriveFalconId, RC::kRightBackSteerFalconId,
                                                  RC::kRightBackCANcoderId, right_back_zero_point);
    m_modules_array = new wpi::array<const SwerveModule*,4>  { m_left_front_swerve_module, m_right_front_swerve_module, 
                                                               m_left_back_swerve_module, m_right_back_swerve_module };


    // The coordinate system is normal mathematical convention with the robot facing
    // along the positive x axis, as shown below.
    //
    //             Y ^
    //   Left Back   |     Left Front
    //               |     
    //               -------->
    //                       X
    //   Right Back        Right Front 
    //
    // Initialise kinematics object containing the positions of each swerve module relative to the centre of the robot
    m_swerve_drive_kinematics = new frc::SwerveDriveKinematics<4>(
        frc::Translation2d( RC::kDriveBaseWheelbaseLength/2.0,  RC::kDriveBaseWheelbaseWidth/2.0), // Left Front
        frc::Translation2d( RC::kDriveBaseWheelbaseLength/2.0, -RC::kDriveBaseWheelbaseWidth/2.0), // Right Front
        frc::Translation2d(-RC::kDriveBaseWheelbaseLength/2.0,  RC::kDriveBaseWheelbaseWidth/2.0), // Left Back
        frc::Translation2d(-RC::kDriveBaseWheelbaseLength/2.0, -RC::kDriveBaseWheelbaseWidth/2.0)  // Right Back
    );

    // Initialise odometry from the kinematics, pigeon and starting pose
    m_swerve_drive_odometry = new frc::SwerveDriveOdometry<4>(*m_swerve_drive_kinematics,
                                                            m_pigeon_imu->GetRotation2d(), 
                                                                {m_left_front_swerve_module->GetPosition(),
                                                                m_right_front_swerve_module->GetPosition(),
                                                                m_left_back_swerve_module->GetPosition(),
                                                                m_right_back_swerve_module->GetPosition()},
                                                            frc::Pose2d{0_m, 0_m, 0_deg});

    std::cout << "Odometry Setup pigeon " <<  m_pigeon_imu->GetRotation2d().Degrees().value() <<
         " angle " << m_swerve_drive_odometry->GetPose().Rotation().Degrees().value() << "\n";                                                        

    // Load and log the steering calibration, and calibrate the steering encoders such that they are zero when pointing forwards
    SwerveSteeringCalibration calibration;
    if (calibration.Load()) {
        std::cout << "Loaded Steering Calibration\n";
        calibration.Log();

        m_left_front_swerve_module->SetSteerEncoderCalibration(calibration.GetLeftFrontAngle());
        m_right_front_swerve_module->SetSteerEncoderCalibration(calibration.GetRightFrontAngle());
        m_left_back_swerve_module->SetSteerEncoderCalibration(calibration.GetLeftBackAngle());
        m_right_back_swerve_module->SetSteerEncoderCalibration(calibration.GetRightBackAngle());
    }

    // Setup the default command so that we can drive using the controller
    SetDefaultCommand(frc2::RunCommand(
        [this] {
        DoJoystickControl();
        }, { this }));
    
    // Create shuffleboard tab for displaying pose estimation and vision output
    m_shuffleboard->DrivebaseTabSetup();
    m_shuffleboard->VisionTabSetup();

    // Setup the vision system
    m_vision = new Vision(this);
    m_vision->Setup();
}

void SwerveDrivebase::Shutdown() {
    // Nothing required
}

void SwerveDrivebase::TeleopInit() {
    // Nothing required
}

void SwerveDrivebase::DisabledInit() {
    // Nothing required
}

void SwerveDrivebase::SimulationInit() {
    // Setup the vision simulation
    m_vision->SetupSimulation();
}


//==============================================================================
// Drivebase State

const frc::Pose2d SwerveDrivebase::GetPose() const {
    return m_vision->GetPose();
//    return m_swerve_estimator->GetEstimatedPosition();
}

bool SwerveDrivebase::GetFieldRelative() const {
    return m_field_relative;
}

frc::ChassisSpeeds SwerveDrivebase::GetChassisSpeeds() {
    wpi::array<frc::SwerveModuleState, 4U> module_states {
        m_left_front_swerve_module->GetState(),
        m_right_front_swerve_module->GetState(),
        m_left_back_swerve_module->GetState(),
        m_right_back_swerve_module->GetState()
    };
    return m_swerve_drive_kinematics->ToChassisSpeeds(module_states);
}

frc::Rotation2d SwerveDrivebase::GetPigeonRotation() const {
    return m_pigeon_imu->GetRotation2d();
}

units::degree_t SwerveDrivebase::GetPigeonHeading() const {
    return units::degree_t(m_pigeon_imu->GetAngle());
}

units::degree_t SwerveDrivebase::GetPitch() const {
    return m_pigeon_imu->GetRotation3d().Y();
}

units::degree_t SwerveDrivebase::GetRoll() const {
    return m_pigeon_imu->GetRotation3d().X();
}

units::inch_t SwerveDrivebase::GetDistanceToTarget() const {
    units::inch_t distance = 0_in;
    frc::Translation2d target_from_zero;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        target_from_zero = frc::Translation2d(16.3_in, 218.4_in);
    } else {
        target_from_zero = frc::Translation2d(652.25_in - 16.3_in, 218.4_in);
    }
    frc::Pose2d pose = GetPose();
    distance = units::math::sqrt(units::math::pow<2>(pose.X()-target_from_zero.X()) + units::math::pow<2>(pose.Y()-target_from_zero.Y()));

    return distance;
}


//==============================================================================
// Operations

void SwerveDrivebase::Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed,
                            units::radians_per_second_t rotation, bool field_relative) {
    Drive(frc::ChassisSpeeds{x_speed, y_speed, rotation}, field_relative);
}

void SwerveDrivebase::Drive(const frc::ChassisSpeeds& chassis_speeds, bool field_relative) {
    // Get the robot relatice chassis speeds. This is preserved for logging.
    if (field_relative) {
        m_drive_robot_speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassis_speeds, GetPose().Rotation());
    } else {
        m_drive_robot_speed = chassis_speeds;
    }

    // Convert the chassis sppeeds into required states for each swerve module
    auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(m_drive_robot_speed);

    // Normalise the wheel speeds so none are above the maximum speed
    m_swerve_drive_kinematics->DesaturateWheelSpeeds(&states, RC::kMaxSpeed);

    // Separate the state of each module
    auto [fl, fr, bl, br] = states;

    // Set the desired state of each module from the desired drivebase kinematics
    m_left_front_swerve_module->SetDesiredState(fl, false);
    m_right_front_swerve_module->SetDesiredState(fr, false);
    m_left_back_swerve_module->SetDesiredState(bl, false);
    m_right_back_swerve_module->SetDesiredState(br, false);
}

void SwerveDrivebase::ResetPose(const frc::Pose2d& pose) {
    // Reset the pose in the odometry. This requies the current gyro angle
    m_swerve_drive_odometry->ResetPosition(m_pigeon_imu->GetRotation2d(),
                                                {m_left_front_swerve_module->GetPosition(),
                                                m_right_front_swerve_module->GetPosition(),
                                                m_left_back_swerve_module->GetPosition(),
                                                m_right_back_swerve_module->GetPosition()},
                                            pose);
    std::cout << "Odometry Reset pigeon " <<  m_pigeon_imu->GetRotation2d().Degrees().value() <<
         " angle " << m_swerve_drive_odometry->GetPose().Rotation().Degrees().value() << "\n";                                                           


    // Reset the pose in the odometry. This requies the current gyro angle
    m_vision->ResetPose(pose);
}

void SwerveDrivebase::AlignWheelsToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {

    // Gets the state of each swerve module from the drivebase kinematics. 
    // If field relative is true, the x, y, rotation and pigeon values are used to generate robot-relative values first                           
    // auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
    //     field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //                         x_speed, y_speed, 0_rad_per_s, m_pigeon_imu->GetRotation2d())
    //                 : frc::ChassisSpeeds{x_speed, y_speed, 0_rad_per_s});
    units::degree_t toe_in_angle = m_shuffleboard->GetToeInAngle();
    auto states = CalculateToeInSwerveModuleStates(x_speed, y_speed, toe_in_angle);

    // Set the speeds to 0. Angles are all different if toe-in is non-zero.
    states[0].speed = 0_mps;
    states[1].speed = 0_mps;
    states[2].speed = 0_mps;
    states[3].speed = 0_mps;

    // Set the desired state of each module allowing dry steering
    m_left_front_swerve_module->SetDesiredState(states[0], true);
    m_right_front_swerve_module->SetDesiredState(states[1], true);
    m_left_back_swerve_module->SetDesiredState(states[2], true);
    m_right_back_swerve_module->SetDesiredState(states[3], true);
}

bool SwerveDrivebase::AreWheelsAlignedToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {
    // Gets the state of each swerve module from the drivebase kinematics. 
    // If field relative is true, the x, y, rotation and pigeon values are used to generate robot-relative values first                           
    // auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
    //     field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //                         x_speed, y_speed, 0_rad_per_s, m_pigeon_imu->GetRotation2d())
    //                 : frc::ChassisSpeeds{x_speed, y_speed, 0_rad_per_s});
    units::degree_t toe_in_angle = m_shuffleboard->GetToeInAngle();
    auto states = CalculateToeInSwerveModuleStates(x_speed, y_speed, toe_in_angle);
    auto [fl, fr, bl, br] = states;

    frc::SwerveModuleState fl_state = m_left_front_swerve_module->GetState();
    frc::SwerveModuleState fr_state = m_right_front_swerve_module->GetState();
    frc::SwerveModuleState bl_state = m_left_back_swerve_module->GetState();
    frc::SwerveModuleState br_state = m_right_back_swerve_module->GetState();

    units::degree_t fl_delta = KoalafiedUtilities::NormaliseMotorAngle(KoalafiedUtilities::NormaliseAngleDiff(fl_state.angle.Degrees(), fl.angle.Degrees()));
    units::degree_t fr_delta = KoalafiedUtilities::NormaliseMotorAngle(KoalafiedUtilities::NormaliseAngleDiff(fr_state.angle.Degrees(), fr.angle.Degrees()));
    units::degree_t bl_delta = KoalafiedUtilities::NormaliseMotorAngle(KoalafiedUtilities::NormaliseAngleDiff(bl_state.angle.Degrees(), bl.angle.Degrees()));
    units::degree_t br_delta = KoalafiedUtilities::NormaliseMotorAngle(KoalafiedUtilities::NormaliseAngleDiff(br_state.angle.Degrees(), br.angle.Degrees()));

    // std::cout
    //     << "FL delta " << fl_delta.value()
    //     << "FR delta " << fr_delta.value()
    //     << "BL delta " << bl_delta.value()
    //     << "BR delta " << br_delta.value() << "\n";
    const units::degree_t ANGLE_TOLERANCE = 8.0_deg;
    return fl_delta < ANGLE_TOLERANCE && fr_delta < ANGLE_TOLERANCE &&
           bl_delta < ANGLE_TOLERANCE && br_delta < ANGLE_TOLERANCE;
}

void SwerveDrivebase::SlowDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {
    units::degree_t toe_in_angle = m_shuffleboard->GetToeInAngle();
    auto states = CalculateToeInSwerveModuleStates(x_speed, y_speed, toe_in_angle);

    // Normalise the wheel speeds so none are above the maximum speed
    m_swerve_drive_kinematics->DesaturateWheelSpeeds(&states, RC::kMaxSpeed);

    // Set the desired state of each module allowing dry steering
    m_left_front_swerve_module->SetDesiredState(states[0], true);
    m_right_front_swerve_module->SetDesiredState(states[1], true);
    m_left_back_swerve_module->SetDesiredState(states[2], true);
    m_right_back_swerve_module->SetDesiredState(states[3], true);
}

void SwerveDrivebase::LockWheels() {
    frc::SwerveModuleState fl_state;
    frc::SwerveModuleState fr_state;
    frc::SwerveModuleState bl_state;
    frc::SwerveModuleState br_state;
    fl_state.angle = frc::Rotation2d(-45_deg);
    fr_state.angle = frc::Rotation2d( 45_deg);
    bl_state.angle = frc::Rotation2d( 45_deg);
    br_state.angle = frc::Rotation2d(-45_deg);
    m_left_front_swerve_module->SetDesiredState(fl_state,  true);
    m_right_front_swerve_module->SetDesiredState(fr_state, true);
    m_left_back_swerve_module->SetDesiredState(bl_state, true);
    m_right_back_swerve_module->SetDesiredState(br_state, true);
}

void SwerveDrivebase::SetBrakeMode(bool brake) {
    m_left_front_swerve_module->SetBrakeMode(brake);
    m_right_front_swerve_module->SetBrakeMode(brake);
    m_left_back_swerve_module->SetBrakeMode(brake);
    m_right_back_swerve_module->SetBrakeMode(brake);
}


//==============================================================================
// Special Development Operations

void SwerveDrivebase::CharacterisationDrive(double x_speed_fraction, double y_speed_fraction,
                                            units::radians_per_second_t rotation, bool field_relative) {

    units::meters_per_second_t x_speed = RC::kMaxSpeed * x_speed_fraction;
    units::meters_per_second_t y_speed = RC::kMaxSpeed * y_speed_fraction;
    
    double speed_fraction = ::sqrt(x_speed_fraction * x_speed_fraction + y_speed_fraction * y_speed_fraction);

    // Gets the state of each swerve module from the drivebase kinematics. 
    // If field relative is true, the x, y, rotation and pigeon values are used to generate robot-relative values first                           
    auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
        field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            x_speed, y_speed, rotation, GetPose().Rotation())
                    : frc::ChassisSpeeds{x_speed, y_speed, rotation});

    // Normalise the wheel speeds so none are above the maximum speed
    m_swerve_drive_kinematics->DesaturateWheelSpeeds(&states, RC::kMaxSpeed);

    // Separate the state of each module
    auto [fl, fr, bl, br] = states;

    // Set the desired state of each module from the desired drivebase kinematics
    m_left_front_swerve_module->CharacterisationDrive(fl.angle, speed_fraction);
    m_right_front_swerve_module->CharacterisationDrive(fr.angle, speed_fraction);
    m_left_back_swerve_module->CharacterisationDrive(bl.angle, speed_fraction);
    m_right_back_swerve_module->CharacterisationDrive(br.angle, speed_fraction);
}


//==============================================================================
// Joystick Control

void SwerveDrivebase::DoJoystickControl() {

    if (!m_controller->IsConnected()) return;

    if (m_controller->GetBButton()) {
        DoState(State::Target);
    } else {
        DoState(State::Idle);
        
        // DoTuningModuleJoystickControl(m_left_front_swerve_module);
        // return;

        // Whenever we are manually driving the motors are set to coast, unless the robot is tilted
        // by more that 5 degrees, in which case we use brake mode. This makes the robot much easier
        // to control when trying to creep up the charging station (when doing a mutliple climb with
        // alliance partners in particular).
        // SetBrakeMode(units::math::abs(GetRoll()) > 5_deg || units::math::abs(GetPitch()) > 5_deg);

        // Special driver commands when holding the left bumper
        if (m_controller->GetLeftBumper()) {
            // Right bumper resets the steering encoder
            // if (m_controller->GetRightBumperReleased()) {
            //     m_left_front_swerve_module->ResetSteerEncoder();
            //     m_right_front_swerve_module->ResetSteerEncoder();
            //     m_left_back_swerve_module->ResetSteerEncoder();
            //     m_right_back_swerve_module->ResetSteerEncoder();
            // }

            // A button saves the steering calibration
            if (m_controller->GetAButtonReleased()) {
                SaveSteeringCalibrationFromCurrentPosition();
            }

        }

        // A button toggles field relative
        if (m_controller->GetAButtonReleased()) {
            m_field_relative = !m_field_relative;
        }

        // X button toggles april tags
        if (m_controller->GetXButtonReleased()) {
            m_vision->SetUpdatePoseEnabled(!m_vision->GetUpdatePoseEnabled());
        }

        // Y button resets the pose
        if (m_controller->GetYButtonReleased()) {
            frc::Pose2d pose = GetPose();
            frc::Rotation2d rotation(0_deg);
            if (KoalafiedUtilities::IsBlueAlliance()) rotation = frc::Rotation2d(180_deg);
            ResetPose(frc::Pose2d(pose.Translation(), rotation));
        }

        /* Do vision targetting logging then the right bumper is held
        if (m_controller->GetRightBumperPressed()) {
            m_vision->StartSpeakerTargeting();
        }
        if (m_controller->GetRightBumper()) {
            double ambiguity;
            units::degree_t angle;
            units::meter_t distance;
            units::meter_t distance_3d;
            m_vision->GetSpeakerTargetInfo(ambiguity, angle, distance, distance_3d);
        }
        if (m_controller->GetRightBumperReleased()) {
            m_vision->StopSpeakerTargeting();
        }
        */

        // If the 'start' button has just been pressed down do align to amp
        if (m_controller->GetStartButtonPressed()) {
            // Create a path following command to move the robot from its current position to directly in front
            // of the amp. This will fail if the robot is tool far away.
            std::optional<frc2::CommandPtr> align_command = AlignToAmpCommand::DoAlignToAmp(*this);
            if (align_command) {
                // Expand the command to only continue while the 'start' button is held down and to do haptic
                // feedback when the path is complete. Note that we have to 'move' the command because it is
                // single point of ownership (it wraps a std::unique_ptr).
                m_align_to_amp_command = std::move(align_command.value())
                                                    .OnlyWhile([this] { return m_controller->GetStartButton(); })
                                                    .AndThen([this] { return m_haptic_controller->DoContinuousFeedback(1.0, 1.0); });

                // Schedule the command and return. The command with be controlling the drivebase until the 'start'
                // button is release (and this function won't be called again until then).
                m_align_to_amp_command->Schedule();
                return;                                      
            } else {
                // The robot was too far away from the amp, so do haptic feedback to indicate failure
                m_haptic_controller->DoContinuousFeedback(1.0, 1.0);
            }
        }

        // If the driver holds the B button lock the wheels and do nothing else
        // if (m_controller->GetBButton()) {
        //     LockWheels();
        //     return;
        // }

        // Get the values for X & Y speed and rotation. Each of these is processed with the following
        // steps:
        //      1. Apply a 'deadband' to allow for the joystick not returning to centre when released.
        //      2. 'Square' the inputs so that small joystick movements give delicate slow control.
        //      3. Rate limit the control so that so that the value does not change too quickly with time.

        // X & Y speed come from the left joystick. Each is negated to convert the joystick convention (left negative)
        // to the mathematical coordinates used for kinematics (see SwerveDrivebase::Setup() for a diagram).
        double x_joystick = -frc::ApplyDeadband(m_controller->GetLeftY(), RC::kJoystickDeadzone);
        x_joystick = KoalafiedUtilities::PowerAdjust(x_joystick, 2.0);
        units::meters_per_second_t x_speed = m_xspeed_limiter.Calculate(x_joystick) * RC::kMaxSpeed;

        double y_joystick = -frc::ApplyDeadband(m_controller->GetLeftX(), RC::kJoystickDeadzone);
        y_joystick = KoalafiedUtilities::PowerAdjust(y_joystick, 2.0);
        units::meters_per_second_t y_speed = m_yspeed_limiter.Calculate(y_joystick) * RC::kMaxSpeed;

        // Rotation comes from the right joystick x direction. It is negated to convert the joystick convention (left negative)
        // to the mathematical coordinates used for kinematics (see SwerveDrivebase::Setup() for a diagram).
        double rotate_joystick = -frc::ApplyDeadband(m_controller->GetRightX(), RC::kJoystickDeadzone);
        rotate_joystick = KoalafiedUtilities::PowerAdjust(rotate_joystick, 2.0);
        units::radians_per_second_t rotate_speed = m_rotation_limiter.Calculate(rotate_joystick) * RC::kMaxAngularSpeed;

        // Calculate a slowdown factor to all movement. This defaults to whatever the drive is happy with and that is
        // used for matches. Can be set less for testing or demos.
        double slowdown_factor = m_drive_shuffleboard == nullptr ? RC::kDefaultDrivebaseSlowDownFactor
                                                                 : m_drive_shuffleboard->GetSlowdownFactor();
        slowdown_factor = std::clamp(slowdown_factor, 0.1, 1.0);

        // Slow down when the manipulator tells us to
        double manipulator_slowdown = m_manipulator ? m_manipulator->GetDrivebaseSlowdown() : 1.0;
        
        // Adjust all speed by the slowdown factors
        x_speed *= slowdown_factor * manipulator_slowdown;
        y_speed *= slowdown_factor * manipulator_slowdown;
        rotate_speed *= slowdown_factor * manipulator_slowdown;

        // Enable switching to precise driving control using POV buttons when the robot is stationary
        if (x_speed == 0_mps && y_speed == 0_mps && rotate_speed == 0_rad_per_s) {
            // Use the right trigger to set the speed, or default to a slow speed
            // there there is no trigger
            units::meters_per_second_t slow_speed = 0.225_mps;
            const units::meters_per_second_t MAX_SLOW_SPEED = 1.0_mps;
            if (m_controller->GetRightTriggerAxis() != 0.0) {
                slow_speed = MAX_SLOW_SPEED * m_controller->GetRightTriggerAxis();
            }

            switch (m_pov_filter.Filter(m_controller->GetPOV())) {
                case RC::kJoystickPovUp:
                    ManualDriveSlow(slow_speed, 0_mps, false);
                    return;
                case RC::kJoystickPovLeft:
                    ManualDriveSlow(0_mps, slow_speed, false);
                    return;
                case RC::kJoystickPovDown:
                    ManualDriveSlow(-slow_speed, 0_mps, false);
                    return;
                case RC::kJoystickPovRight:
                    ManualDriveSlow(0_mps, -slow_speed, false);
                    return;
            }
        }

        // Not doing slow driving so clear aligned flag
        m_manual_drive_slow_aligned = false;
        m_manual_drive_slow_aligned_timer.Reset();
        m_manual_drive_slow_aligned_timer.Start();

        // If we are not the blue alliance then drivers position is rotated 180 degrees and so for
        // field relative we rotate 180 degrees.
        if (!KoalafiedUtilities::IsBlueAlliance()) {
            x_speed = -x_speed;
            y_speed = -y_speed;
        }

        // Do normal (not slow) driving
    //    Drive(x_speed, y_speed, rotate_speed, m_field_relative);
        Drive(x_speed, y_speed, rotate_speed, true);
    }
}

void SwerveDrivebase::ManualDriveSlow(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {

    // Check if the wheels are aligned to the desired direction. Slow driving only starts once the wheels
    // are aligned.
    if (AreWheelsAlignedToDrive(x_speed, y_speed, field_relative)) {
        // If this when the wheels are first aligned log how long it took. This is very helpful when
        // testing how fast the the swerve modules are turning.
        if (!m_manual_drive_slow_aligned) {
            m_manual_drive_slow_aligned = true;
            std::cout << "Aligning slow took " << m_manual_drive_slow_aligned_timer.Get().value() << "s\n";
        }

        // Start driving at the slow speed
        SlowDrive(x_speed, y_speed, field_relative);
    }
    else {
        // Wheel are not aligned so turn then to the aligne position 
        AlignWheelsToDrive(x_speed, y_speed, field_relative);
    }
}

void SwerveDrivebase::DoTuningJoystickControl() {
    // This is a tuning function to use only under careful conditions when first getting the drivebase
    // running. To use it the code must be modified to actually call it.

    // If the POV is held do test driving of a single module
    int pov = m_controller->GetPOV();
    if (pov != -1) {
        switch (pov) {
            case   0: DoTuningModuleJoystickControl(m_left_front_swerve_module); break;
            case  90: DoTuningModuleJoystickControl(m_right_front_swerve_module); break;
            case 180: DoTuningModuleJoystickControl(m_left_back_swerve_module); break;
            case 270: DoTuningModuleJoystickControl(m_right_back_swerve_module); break;
        }
        return;
    }

    // Use the right stick to drive the robot with standard tuning control. This must only be done when the
    // wheels are aligned!!
    double speed = -m_controller->GetRightY() * 0.3;
    double closed_loop = m_controller->GetLeftBumper();
    double max_rmp = 5000;

    m_left_front_swerve_module->TuneDriveTalonFX(speed, max_rmp, closed_loop);
    m_right_front_swerve_module->TuneDriveTalonFX(speed, max_rmp, closed_loop);
    m_left_back_swerve_module->TuneDriveTalonFX(speed, max_rmp, closed_loop);
    m_right_back_swerve_module->TuneDriveTalonFX(speed, max_rmp, closed_loop);

    // Use the left stick to drive steering with standard tuning control. This must only be done when the
    // wheels are off the floor
    double steer_speed = -m_controller->GetLeftX() * 0.3;
    closed_loop = m_controller->GetRightBumper();

    m_left_front_swerve_module->TuneSteerTalonFX(steer_speed, max_rmp, closed_loop);
    m_right_front_swerve_module->TuneSteerTalonFX(steer_speed, max_rmp, closed_loop);
    m_left_back_swerve_module->TuneSteerTalonFX(steer_speed, max_rmp, closed_loop);
    m_right_back_swerve_module->TuneSteerTalonFX(steer_speed, max_rmp, closed_loop);
}

void SwerveDrivebase::DoTuningModuleJoystickControl(SwerveModule* swerve_module) {

    // If buttons are down rotate the module to 90 degree increments
    // if (m_controller->GetYButton()) {
    //     swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(0.0_rad)}, false);
    //     return;
    // }
    // if (m_controller->GetXButton()) {
    //     swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * 0.5_rad)}, false);
    //     return;
    // }
    // if (m_controller->GetAButton()) {
    //     swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * 1.0_rad)}, false);
    //     return;
    // }
    // if (m_controller->GetBButton()) {
    //     swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * -0.5_rad)}, false);
    //     return;
    // }

    int pov = m_controller->GetPOV();
    if (pov != -1) {
        units::meters_per_second_t speed = 1_mps;
        switch (pov) {
            case   0: swerve_module->SetDesiredState({speed, frc::Rotation2d(0_deg)}, true); break;
            case  90: swerve_module->SetDesiredState({speed, frc::Rotation2d(90_deg)}, true); break;
            case 180: swerve_module->SetDesiredState({speed, frc::Rotation2d(180_deg)}, true); break;
            case 270: swerve_module->SetDesiredState({speed, frc::Rotation2d(270_deg)}, true); break;
        }
        return;
    }


    // Otherwise just manual drive  with the right stick
    //double test_drive = m_controller->GetRightY();
    double test_drive = 0;
    double test_steer = m_controller->GetRightX();
    test_drive = KoalafiedUtilities::PowerAdjust(test_drive, 2.0);
    swerve_module->ManualDriveMotors(test_drive, test_steer);
}


//==============================================================================
// State Control

void SwerveDrivebase::DoState(State state) {
    if (m_state != state) {
        ExitState();
        m_state = state;
        EnterState();
    }
    UpdateState();
}

void SwerveDrivebase::EnterState() {
    switch (m_state) {
    case State::Idle:
        break;
    case State::Target: EnterTargetingState();
        break;
    }
}

void SwerveDrivebase::UpdateState() { 
    switch (m_state) {
    case State::Idle:
        break;
    case State::Target: UpdateTargetingState();
        break;
    }
}

void SwerveDrivebase::ExitState() {
    switch (m_state) { 
        case State::Idle:
            break;
        case State::Target: ExitTargetingState();
            break;
    }
}

void SwerveDrivebase::EnterTargetingState() {
    // Determine what type of targeting to do
    double ambiguity;
    units::degree_t angle;
    units::meter_t distance;
    if (m_vision->GetSpeakerTargetInfo(ambiguity, angle, distance)) {
        // Speaker is visible so target that
        m_target_state = TargetState::SpeakerTargeting;
        m_vision->StartSpeakerTargeting();
    } else if (m_vision->GetLobTargetInfo(angle, distance)) {
        // A lob shot is possible so target that
        m_target_state = TargetState::LobTargeting;
    } else {
        // No targetting is possible to indicate failure
        m_target_state = TargetState::Failed;
        m_haptic_controller->DoFailureFeedback();
        Drive(0_mps, 0_mps, 0_rad_per_s, false);
    }

    m_pointing_at_target = false;
    SetBrakeMode(true);
}

void SwerveDrivebase::UpdateTargetingState() {
    // Get the target information depending on the current target state
    bool target_ok; // Whether the target is found 
    units::degree_t angle; // Angle the robot needs to turn to get to the target
    units::meter_t distance;
    units::degree_t error_threshold; // Error that the robot angle must be within to count as no target
    switch (m_target_state) {
        case TargetState::SpeakerTargeting:
            double ambiguity;
            target_ok = m_vision->GetSpeakerTargetInfo(ambiguity, angle, distance);
            if (target_ok) {
                m_manipulator->DoTargetingPrefire(distance);
                units::inch_t MARGIN = 7_in;
                error_threshold = units::math::atan2(MARGIN, distance);
            }
            break;
        case TargetState::LobTargeting:
            target_ok = m_vision->GetLobTargetInfo(angle, distance);
            if (target_ok) {
                m_manipulator->DoLobbingPrefire(distance);
                error_threshold = 3_deg;
            }
            break;
        case TargetState::OnTarget:
        case TargetState::Failed:
        default:
            // Once we are on target, or have failed to find the target, there is nothing to do so just return
            return;
    }

    // If the target is ok drive towards it
    if (target_ok) {
        // If outside the error threshold keep turning towards the target with simple P gain
        m_pointing_at_target = units::math::abs(angle) < error_threshold;
        if (!m_pointing_at_target) {
            units::second_t TURN_TIME = 0.5_s;
            units::degrees_per_second_t omega = -angle / TURN_TIME;
            units::degrees_per_second_t MAX_OMEGA = 45_deg / 1_s;
            if (omega > MAX_OMEGA) omega = MAX_OMEGA;
            if (omega < -MAX_OMEGA) omega = -MAX_OMEGA;

            Drive(0_mps, 0_mps, omega, false);
        } else {
            // Point at the target so stop rotating
            Drive(0_mps, 0_mps, 0_rad_per_s, false);

            // Test that the shooter is ready
            if (m_manipulator->PrefireReady()) {
                // Fully ready to shoot so change state and give haptic feedback that we can fire now
                m_target_state = TargetState::OnTarget;
                std::cout << "Angle: " << angle.value() << " Threshold: " << error_threshold.value() << "\n";
                m_haptic_controller->DoSuccessFeedback();
                m_manipulator->GetHapticController()->DoSuccessFeedback();
            }
        }
    } else {
        // Target has been lost so go to failure
        m_target_state = TargetState::Failed;
        m_manipulator->StopPrefire();
        m_haptic_controller->DoFailureFeedback();
        Drive(0_mps, 0_mps, 0_rad_per_s, false);
    }
}

void SwerveDrivebase::ExitTargetingState() {
    m_manipulator->StopPrefire();
    m_vision->StopSpeakerTargeting();
    SetBrakeMode(false);
    m_pointing_at_target = false;
}


//==========================================================================
// Toe In Calculation

wpi::array<frc::SwerveModuleState, 4> SwerveDrivebase::CalculateToeInSwerveModuleStates(units::meters_per_second_t x_speed,
        units::meters_per_second_t y_speed, units::degree_t toe_in_angle) {

    //wpi::array<SwerveModuleState, 4> moduleStates(wpi::empty_array);
    wpi::array<frc::SwerveModuleState, 4> module_states =
        m_swerve_drive_kinematics->ToSwerveModuleStates(frc::ChassisSpeeds{x_speed, y_speed, 0_rad_per_s});

    const units::degree_t MAX_TOE_IN_ANGLE = 20_deg;
    if (toe_in_angle < 0_deg) toe_in_angle = 0_deg;
    if (toe_in_angle > MAX_TOE_IN_ANGLE) toe_in_angle = MAX_TOE_IN_ANGLE;

    //
    // The coordinate system is normal mathematical convention with the robot facing
    // along the positive x axis, as shown below.
    //
    //                Y ^
    //   Left Back - 2  |     Left Front - 0
    //                  |     
    //                  -------->
    //                          X
    //   Right Back - 3       Right Front - 1

    if (y_speed == 0_mps) {
        if (x_speed < 0_mps) toe_in_angle = -toe_in_angle;
        module_states[0].angle = module_states[0].angle.RotateBy(-toe_in_angle);
        module_states[1].angle = module_states[1].angle.RotateBy(+toe_in_angle);
        module_states[2].angle = module_states[2].angle.RotateBy(-toe_in_angle);
        module_states[3].angle = module_states[3].angle.RotateBy(+toe_in_angle);
    } else if (x_speed == 0_mps) {
        if (y_speed < 0_mps) toe_in_angle = -toe_in_angle;
        module_states[0].angle = module_states[0].angle.RotateBy(+toe_in_angle);
        module_states[1].angle = module_states[1].angle.RotateBy(+toe_in_angle);
        module_states[2].angle = module_states[2].angle.RotateBy(-toe_in_angle);
        module_states[3].angle = module_states[3].angle.RotateBy(-toe_in_angle);
    }

    // std::cout << "[" << module_states[0].speed.value() << " - " << module_states[0].angle.Degrees().value() <<"]";
    // std::cout << "[" << module_states[1].speed.value() << " - " << module_states[1].angle.Degrees().value() <<"]";
    // std::cout << "[" << module_states[2].speed.value() << " - " << module_states[2].angle.Degrees().value() <<"]";
    // std::cout << "[" << module_states[3].speed.value() << " - " << module_states[3].angle.Degrees().value() <<"]\n";
    return module_states;
}


//==============================================================================
// Steering Calibration

void SwerveDrivebase::SaveSteeringCalibrationFromCurrentPosition() {
    // Get the calibration angle from the swerve moduels
    SwerveSteeringCalibration calibration(m_left_front_swerve_module->GetAbsoluteSteerEncoderPosition(),
                                          m_right_front_swerve_module->GetAbsoluteSteerEncoderPosition(),
                                          m_left_back_swerve_module->GetAbsoluteSteerEncoderPosition(),
                                          m_right_back_swerve_module->GetAbsoluteSteerEncoderPosition());

    // Log the values and save them to the calibration file
    std::cout << "Saving Steering Calibration\n";
    calibration.Log();
    calibration.Save();
}
