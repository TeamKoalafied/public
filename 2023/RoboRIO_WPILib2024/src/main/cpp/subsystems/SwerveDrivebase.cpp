//==============================================================================
// SwerveDrivebase.cpp
//==============================================================================

#include "SwerveDrivebase.h"

#include "Manipulator.h"
#include "Vision.h"
#include "SwerveDrivebaseShuffleboard.h"

#include "../RobotConfiguration.h"
#include "../commands/TestTurnCommand.h"
#include "../util/HapticController.h"
#include "../util/KoalafiedUtilities.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SwerveControllerCommand.h>

// #include <pathplanner/lib/PathPlanner.h>
// #include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <units/length.h>

#include <fstream>
#include <iostream>
#include <iomanip>

namespace RC = RobotConfiguration;

//==============================================================================
// Construction 

SwerveDrivebase::SwerveDrivebase(Manipulator* manipulator) {
    m_manipulator = manipulator;

    // Set all timers to only show processing warnings, with generous limits    
    //m_joystick_timer.SetMode((PeriodicTimer::Mode)(PeriodicTimer::AutoMilliseconds | PeriodicTimer::ShowProcessingWarnings), 5000);
    m_joystick_timer.SetPeriodicLimits(30_ms, 20_ms);
    //m_simulation_timer.SetMode((PeriodicTimer::Mode)(PeriodicTimer::AutoMilliseconds | PeriodicTimer::ShowProcessingWarnings), 5000);
    m_simulation_timer.SetPeriodicLimits(30_ms, 20_ms);
    //m_update_timer.SetMode((PeriodicTimer::Mode)(PeriodicTimer::AutoMilliseconds | PeriodicTimer::ShowProcessingWarnings), 5000);
    m_periodic_timer.SetPeriodicLimits(30_ms, 20_ms);
}

// Destructor
SwerveDrivebase::~SwerveDrivebase() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void SwerveDrivebase::Periodic() {
    m_periodic_timer.PeriodicStart();

    // Update the odometry from the pigeon values and swerve module state 
    const wpi::array<frc::SwerveModulePosition,4U> module_positions{
        m_left_front_swerve_module->GetPosition(),
        m_right_front_swerve_module->GetPosition(),
        m_left_back_swerve_module->GetPosition(),
        m_right_back_swerve_module->GetPosition()
    };
    m_swerve_drive_odometry->Update(m_pigen_imu->GetRotation2d(), module_positions);

    m_vision->Update(wpi::array<double, 3>{0.01, 0.01, 0.01});
    // Update the vision
    //m_latest_photon_pose = m_photon_estimator->Update();
    // if (m_latest_photon_pose) {
    //     // If there is a result use it to update the estimated position, as long as the area is large enough
    //     m_latest_photon_result = m_photon_estimator->GetCamera().GetLatestResult();
    //     if (m_latest_photon_result.GetBestTarget().GetArea() > 1) {
    //         // Using the time from the estimated pose in the simulator causes a crash in AddVisionMeasurement()
    //         // so to prevent that we just use the current time.
    //         //  m_swerve_estimator->AddVisionMeasurement(photon_pose.value().estimatedPose.ToPose2d(), photon_pose.value().timestamp);
    //         m_swerve_estimator->AddVisionMeasurement(m_latest_photon_pose.value().estimatedPose.ToPose2d(),
    //                                                  units::microsecond_t(wpi::Now()));
    //        m_latest_photon_result.GetTimestamp();                                         
    //     }
    // }
    units::degree_t lf_scrub = m_left_front_swerve_module->GetDesiredState().angle.Degrees() - module_positions[0].angle.Degrees();
    m_left_front_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(lf_scrub));
    units::degree_t rf_scrub = m_right_front_swerve_module->GetDesiredState().angle.Degrees() - module_positions[1].angle.Degrees();
    m_right_front_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(rf_scrub));
    units::degree_t lb_scrub = m_left_back_swerve_module->GetDesiredState().angle.Degrees() - module_positions[2].angle.Degrees();
    m_left_back_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(lb_scrub));
    units::degree_t rb_scrub = m_right_back_swerve_module->GetDesiredState().angle.Degrees() - module_positions[3].angle.Degrees();
    m_right_back_swerve_module->SetScrub(KoalafiedUtilities::NormaliseMotorAngle(rb_scrub));
    // Update the haptic controller and shuffleboard. Note shuffle board is updated last as it
    // display state that we have just calculate.
    m_haptic_controller->Periodic();
    m_shuffleboard->UpdateShuffleboard();
    
    m_periodic_timer.PeriodicEnd();
}

void SwerveDrivebase::SimulationPeriodic() {

    m_simulation_timer.PeriodicStart();

    units::second_t update_time = 0.02_s;

    // Update each of the swerve modules
    m_left_front_swerve_module->UpdateSimulation(update_time);
    m_right_front_swerve_module->UpdateSimulation(update_time);
    m_left_back_swerve_module->UpdateSimulation(update_time);
    m_right_back_swerve_module->UpdateSimulation(update_time);

    // Combine the state of the 4 swerve modules to get the overall velocity and rotation
    // of the robot. Update the Pigeon heading for the current angular velocity over the
    // simulation update time.
    frc::ChassisSpeeds chassis_speeds = GetChassisSpeeds();
    units::degree_t new_angle = m_pigen_imu->GetRotation2d().Degrees() + chassis_speeds.omega * update_time;
    m_pigen_imu->GetSimCollection().SetRawHeading(new_angle.value());

    // Update the vision simulation
    m_vision->UpdateSimulation();

    m_simulation_timer.PeriodicEnd();
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
    m_pigen_imu = new WPI_PigeonIMU(RC::kPigeonImuId);
    m_pigen_imu->SetFusedHeading(0.0, RC::kTalonTimeoutMs);

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
                                                            m_pigen_imu->GetRotation2d(), 
                                                                {m_left_front_swerve_module->GetPosition(),
                                                                m_right_front_swerve_module->GetPosition(),
                                                                m_left_back_swerve_module->GetPosition(),
                                                                m_right_back_swerve_module->GetPosition()},
                                                            frc::Pose2d{0_m, 0_m, 180_deg});

    std::cout << "Odometry Setup pigeon " <<  m_pigen_imu->GetRotation2d().Degrees().value() <<
         " angle " << m_swerve_drive_odometry->GetPose().Rotation().Degrees().value() << "\n";                                                        
    
    // Load and log the steering calibration, and calibrate the steering encoders such that they are zero when pointing forwards
    SteeringCalibration calibration;
    if (LoadSteeringCalibration(calibration)) {
        std::cout << "Loaded Steering Calibration\n";
        LogSteeringCalibration(calibration);

        m_left_front_swerve_module->SetSteerEncoderCalibration(calibration.m_left_front_angle);
        m_right_front_swerve_module->SetSteerEncoderCalibration(calibration.m_right_front_angle);
        m_left_back_swerve_module->SetSteerEncoderCalibration(calibration.m_left_back_angle);
        m_right_back_swerve_module->SetSteerEncoderCalibration(calibration.m_right_back_angle);
    }

    // Setup the default command so that we can drive using the controller
    SetDefaultCommand(frc2::RunCommand(
        [this] {
        DoJoystickControl();
        }, { this }));
    
    // Create shuffleboard tab for displaying pose estimation and vision output
    m_shuffleboard->DrivebaseTabSetup();
    m_shuffleboard->VisionTabSetup();
    m_shuffleboard->DriveTabSetup();

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
}

bool SwerveDrivebase::GetFieldRelative() const {
    return m_field_relative;
}

wpi::array<const SwerveModule*,4> SwerveDrivebase::GetModules() const {
    return {m_left_front_swerve_module, m_right_front_swerve_module, 
            m_left_back_swerve_module, m_right_back_swerve_module};
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

// frc::SwerveDrivePoseEstimator<4>* SwerveDrivebase::GetEstimator() const {
//     return m_swerve_estimator;
// }

frc::Rotation2d SwerveDrivebase::GetPigeonRotation() const {
    return m_pigen_imu->GetRotation2d();
}

units::degree_t SwerveDrivebase::GetPigeonHeading() const {
    return units::degree_t(m_pigen_imu->GetFusedHeading());
}

units::degree_t SwerveDrivebase::GetPitch() const {
    return units::degree_t(m_pigen_imu->GetPitch());
}

units::degree_t SwerveDrivebase::GetRoll() const {
    return units::degree_t(m_pigen_imu->GetRoll());
}


//==============================================================================
// Operations

void SwerveDrivebase::Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed,
                            units::radians_per_second_t rotation, bool field_relative) {
    Drive(frc::ChassisSpeeds{x_speed, y_speed, rotation}, field_relative);
}

void SwerveDrivebase::Drive(const frc::ChassisSpeeds& chassis_speeds, bool field_relative) {
    auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
        field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassis_speeds, GetPose().Rotation())
                       : chassis_speeds);

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
    m_swerve_drive_odometry->ResetPosition(m_pigen_imu->GetRotation2d(),
                                                {m_left_front_swerve_module->GetPosition(),
                                                m_right_front_swerve_module->GetPosition(),
                                                m_left_back_swerve_module->GetPosition(),
                                                m_right_back_swerve_module->GetPosition()},
                                            pose);
    std::cout << "Odometry Reset pigeon " <<  m_pigen_imu->GetRotation2d().Degrees().value() <<
         " angle " << m_swerve_drive_odometry->GetPose().Rotation().Degrees().value() << "\n";                                                           


    // Reset the pose in the odometry. This requies the current gyro angle
    m_vision->ResetPose(pose);
}

void SwerveDrivebase::AlignWheelsToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {

    // Gets the state of each swerve module from the drivebase kinematics. 
    // If field relative is true, the x, y, rotation and pigeon values are used to generate robot-relative values first                           
    auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
        field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            x_speed, y_speed, 0_rad_per_s, m_pigen_imu->GetRotation2d())
                    : frc::ChassisSpeeds{x_speed, y_speed, 0_rad_per_s});

    // Set the speed to 0. All the states are the same, so just use the first one.
    states[0].speed = 0_mps;

    // Set the desired state of each module allowing dry steering
    m_left_front_swerve_module->SetDesiredState(states[0], true);
    m_right_front_swerve_module->SetDesiredState(states[0], true);
    m_left_back_swerve_module->SetDesiredState(states[0], true);
    m_right_back_swerve_module->SetDesiredState(states[0], true);
}

bool SwerveDrivebase::AreWheelsAlignedToDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, bool field_relative) {
    // Gets the state of each swerve module from the drivebase kinematics. 
    // If field relative is true, the x, y, rotation and pigeon values are used to generate robot-relative values first                           
    auto states = m_swerve_drive_kinematics->ToSwerveModuleStates(
        field_relative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            x_speed, y_speed, 0_rad_per_s, m_pigen_imu->GetRotation2d())
                    : frc::ChassisSpeeds{x_speed, y_speed, 0_rad_per_s});
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

frc2::CommandPtr SwerveDrivebase::CreateTrajectoryCommand(const frc::Trajectory& trajectory) {
    frc::ProfiledPIDController<units::radians> thetaController{
    RC::kPThetaController, 0, 0,
    RC::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                    units::radian_t{std::numbers::pi});

    return frc2::cmd::Sequence(
        frc2::InstantCommand([this, trajectory] { ResetPose(trajectory.InitialPose()); }, { this }).ToPtr(),
        frc2::SwerveControllerCommand<4>(
           trajectory, [this]() { return GetPose(); },
            *m_swerve_drive_kinematics,
            frc::PIDController{RC::kPXController, 0, 0},
            frc::PIDController{RC::kPYController, 0, 0}, thetaController,
            [this](auto moduleStates) { SetModuleStates(moduleStates); },
            {this}).ToPtr()
        );
    // return std::move(frc2::InstantCommand([this, trajectory] {ResetPose(trajectory.InitialPose()); }, {this}))
    //         .AndThen(std::move(frc2::SwerveControllerCommand<4>(trajectory, [this]() {return GetPose(); },
    //                     *m_swerve_drive_kinematics,
    //                     frc::PIDController{RC::kPXController, 0, 0},
    //                     frc::PIDController{RC::kPYController, 0, 0}, thetaController,
    //                     [this](auto moduleStates) {SetModuleStates(moduleStates); },
    //                     {this}))
    //         );
}

void SwerveDrivebase::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    m_swerve_drive_kinematics->DesaturateWheelSpeeds(&desiredStates, RC::kMaxSpeed);
    m_left_front_swerve_module->SetDesiredState(desiredStates[0], false);
    m_right_front_swerve_module->SetDesiredState(desiredStates[1], false);
    m_left_back_swerve_module->SetDesiredState(desiredStates[2], false);
    m_right_back_swerve_module->SetDesiredState(desiredStates[3], false);
}

void SwerveDrivebase::SetFollowerModuleStates(wpi::array<SwerveFollowerModuleState, 4> desiredStates) {
    // TODO Desaturate wheel speeds. Maybe do after close loop calculation?

    m_left_front_swerve_module->SetDesiredFollowerState(desiredStates[0]);
    m_right_front_swerve_module->SetDesiredFollowerState(desiredStates[1]);
    m_left_back_swerve_module->SetDesiredFollowerState(desiredStates[2]);
    m_right_back_swerve_module->SetDesiredFollowerState(desiredStates[3]);
}


//==============================================================================
// Joystick Control

void SwerveDrivebase::DoJoystickControl() {

    m_joystick_timer.PeriodicStart();

    // TODO This runs stupidly slow in the simulator (~250ms). Need to see if it also is slow on
    // the robot. Either way it should setting the brake mode should probably cache the current
    // setting.

    // Whenever we are manually driving the motors are set to coast, unless the robot is tilted
    // by more that 5 degrees, in which case we use brake mode. This makes the robot much easier
    // to control when trying to creep up the charging station (when doing a mutliple climb with
    // alliance partners in particular).
    //SetBrakeMode(units::math::abs(GetRoll()) > 5_deg || units::math::abs(GetPitch()) > 5_deg);

    // Special driver commands when holding the left bumper
    if (m_controller->GetLeftBumper()) {
        // Right bumper resets the steering encoder
        if (m_controller->GetRightBumperReleased()) {
            m_left_front_swerve_module->ResetSteerEncoder();
            m_right_front_swerve_module->ResetSteerEncoder();
            m_left_back_swerve_module->ResetSteerEncoder();
            m_right_back_swerve_module->ResetSteerEncoder();
        }

        // A button saves the steering calibration
        if (m_controller->GetAButtonReleased()) {
            SaveSteeringCalibrationFromCurrentPosition();
        }

    }

    // A button toggles field relative
    if (m_controller->GetAButtonReleased()) {
        m_field_relative = !m_field_relative;
    }

    // Y button resets the pose
    if (m_controller->GetYButtonReleased()) {
        frc::Pose2d pose = GetPose();
        ResetPose(frc::Pose2d(pose.Translation(), frc::Rotation2d()));
    }

    // If the driver holds the B button lock the wheels and do nothing else
    if (m_controller->GetBButton()) {
        LockWheels();
        return;
    }

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
    double slowdown_factor = m_shuffleboard->GetSlowdownFactor();
    slowdown_factor = std::clamp(slowdown_factor, 0.1, 1.0);

    // Slow down when the arm is extended
    double arm_slowdown = 1.0;
    if (m_manipulator != nullptr) {
        const double FULL_EXTENSION_SLOWDOWN = 0.3;
        double arm_extension_factor = m_manipulator->GetArmExtension() / RC::kArmMaximumExtensionInch;
        arm_slowdown = (1.0 - arm_extension_factor) + FULL_EXTENSION_SLOWDOWN * arm_extension_factor;
    }

    // Adjust all speed by the slowdown factors
    x_speed *= slowdown_factor * arm_slowdown;
    y_speed *= slowdown_factor * arm_slowdown;
    rotate_speed *= slowdown_factor * arm_slowdown;

    // Enable switching to precise driving control using POV buttons when the robot is stationary
    if (x_speed == 0_mps && y_speed == 0_mps && rotate_speed == 0_rad_per_s) {
        units::meters_per_second_t slow_speed = 0.225_mps;
        if (m_controller->GetRightBumper()) slow_speed *= 4.0;
        switch (m_controller->GetPOV()) {
            case RC::kJoystickPovUp:
                ManualDriveSlow(slow_speed, 0_mps, false);
                return;
            case RC::kJoystickPovUpLeft:
                ManualDriveSlow(slow_speed, slow_speed, false);
                return;
            case RC::kJoystickPovLeft:
                ManualDriveSlow(0_mps, slow_speed, false);
                return;
            case RC::kJoystickPovDownLeft:
                ManualDriveSlow(-slow_speed, slow_speed, false);
                return;
            case RC::kJoystickPovDown:
                ManualDriveSlow(-slow_speed, 0_mps, false);
                return;
            case RC::kJoystickPovDownRight:
                ManualDriveSlow(-slow_speed, -slow_speed, false);
                return;
            case RC::kJoystickPovRight:
                ManualDriveSlow(0_mps, -slow_speed, false);
                return;
            case RC::kJoystickPovUpRight:
                ManualDriveSlow(slow_speed, -slow_speed, false);
                return;
        }
    }

    // Not doing slow driving so clear aligned flag
    m_manual_drive_slow_aligned = false;
    m_manual_drive_slow_aligned_timer.Reset();
    m_manual_drive_slow_aligned_timer.Start();

    // Do normal (not slow) driving
    bool field_relative = !m_controller->GetRightBumper();
    Drive(x_speed, y_speed, rotate_speed, field_relative);

    m_joystick_timer.PeriodicEnd();
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
        Drive(x_speed, y_speed, 0_rad_per_s, field_relative);
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
    if (m_controller->GetYButton()) {
        swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(0.0_rad)}, false);
        return;
    }
    if (m_controller->GetXButton()) {
        swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * 0.5_rad)}, false);
        return;
    }
    if (m_controller->GetAButton()) {
        swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * 1.0_rad)}, false);
        return;
    }
    if (m_controller->GetBButton()) {
        swerve_module->SetDesiredState({0.05_mps, frc::Rotation2d(std::numbers::pi * -0.5_rad)}, false);
        return;
    }

    // Otherwise just manual drive  with the right stick
    double test_drive = m_controller->GetRightY();
    double test_steer = m_controller->GetRightX();
    test_drive = KoalafiedUtilities::PowerAdjust(test_drive, 2.0);
    swerve_module->ManualDriveMotors(test_drive, test_steer);
}



//==============================================================================
// Steering Calibration

void SwerveDrivebase::SaveSteeringCalibrationFromCurrentPosition() {
    // Get the calibration angle from the swerve moduels
    SteeringCalibration calibration;
    calibration.m_left_front_angle = m_left_front_swerve_module->GetAbsoluteSteerEncoderPosition();
    calibration.m_right_front_angle = m_right_front_swerve_module->GetAbsoluteSteerEncoderPosition();
    calibration.m_left_back_angle = m_left_back_swerve_module->GetAbsoluteSteerEncoderPosition();
    calibration.m_right_back_angle = m_right_back_swerve_module->GetAbsoluteSteerEncoderPosition();

    // Log the values and save them to the calibration file
    std::cout << "Saving Steering Calibration\n";
    LogSteeringCalibration(calibration);
    SaveSteeringCalibration(calibration);
}

void SwerveDrivebase::SaveSteeringCalibration(const SteeringCalibration& calibration) {
    // Open the calibration file for writing discarding the existing contents
    std::ofstream calibration_file;
    calibration_file.open(CALIBRATION_FILENAME, std::ios::out | std::ios::trunc);
    if (calibration_file.fail()) {
        std::cout << "ERROR: Failed to open calibration file for writing " << CALIBRATION_FILENAME << "\n";
        return;
    }

    // Write the values one per line, with a comment to explain the format
    calibration_file << "# This file holds the swerve drive calibration with one value per line in\n";
    calibration_file << "# the order left front, right front, left back, right back\n";
    calibration_file << std::setprecision(4) << calibration.m_left_front_angle.value() << "\n";
    calibration_file << std::setprecision(4) << calibration.m_right_front_angle.value() << "\n";
    calibration_file << std::setprecision(4) << calibration.m_left_back_angle.value() << "\n";
    calibration_file << std::setprecision(4) << calibration.m_right_back_angle.value() << "\n";

    std::cout << "Steering calibration  written to " << CALIBRATION_FILENAME << "\n";
}

bool SwerveDrivebase::LoadSteeringCalibration(SteeringCalibration& calibration) {
    // Open the calibration file for reading
    std::ifstream calibration_file;
    calibration_file.open(CALIBRATION_FILENAME, std::ios::in);
    if (calibration_file.fail()) {
        std::cout << "ERROR: Failed to open calibration file for reading" << CALIBRATION_FILENAME << "\n";
        return false;
    }

    // Read the file line by line, looking for 4 calibration angle values
    std::string line;
    int index = 0;
    while (std::getline(calibration_file, line)) {
        // Skip comment lines and empty lines
        if (line[0] == '#') continue;
        if (line.empty()) continue;

        // Try to read the line as a single number. If not valid there is a problem.
        char* end;
        double value = strtod(line.c_str(), &end);
        if (*end != '\0') {
            std::cout << "ERROR: Calibration angle number " << index << " is illegal\n";
            std::cout << line << "\n";
            return false;
        }

        // Assign the number read to the appropriate calibration angle
        switch (index) {
            case 0: calibration.m_left_front_angle = units::degree_t(value); break;
            case 1: calibration.m_right_front_angle = units::degree_t(value); break;
            case 2: calibration.m_left_back_angle = units::degree_t(value); break;
            case 3: calibration.m_right_back_angle = units::degree_t(value); break;
        }
        
        // Move to the next corner. If we have 4 we are done.
        index++;
        if (index == 4) return true;
    }

    // Did not find 4 values
    std::cout << "ERROR: Calibration file only had " << index << " values\n";
    return false;
}

void SwerveDrivebase::LogSteeringCalibration(const SteeringCalibration& calibration) {
    std::cout << "Left Front:  " << calibration.m_left_front_angle.value() << "\n";
    std::cout << "right Front: " << calibration.m_right_front_angle.value() << "\n";
    std::cout << "Left Back:   " << calibration.m_left_back_angle.value() << "\n";
    std::cout << "Right Back:  " << calibration.m_right_back_angle.value() << "\n";
}