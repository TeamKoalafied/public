//==============================================================================
// LogPathPlannerCommand.cpp
//==============================================================================

#include "LogVisionCommand.h"

#include "../subsystems/Mechanisms/Diverter.h"
#include "../subsystems/Mechanisms/Pivot.h"
#include "../subsystems/Mechanisms/Shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/commands/PathfindHolonomic.h>
#include <iostream>

using namespace pathplanner;


//==============================================================================
// Construction

LogVisionCommand::LogVisionCommand(SwerveDrivebase* drivebase, Manipulator* manipulator, 
                                        std::shared_ptr<PathPlannerPath> path, const Parameters parameters) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
    // m_vision = m_drivebase->GetVision();
    m_auto_paths.push_back(path);
    m_parameters = parameters;
}

LogVisionCommand::LogVisionCommand(SwerveDrivebase* drivebase, Manipulator* manipulator,
                                             std::vector<std::shared_ptr<PathPlannerPath>> auto_paths, const Parameters parameters)
{
    m_drivebase = drivebase;
    m_manipulator = manipulator;
    // m_vision = m_drivebase->GetVision();
    m_auto_paths = auto_paths;
    m_parameters = parameters;
}

LogVisionCommand::LogVisionCommand(const LogVisionCommand&) {
    // This copy constructor needs to exist for the code to build but should not be called. Not sure
    // what is going on.
    assert(false);
}

LogVisionCommand::~LogVisionCommand() {
}


//==============================================================================
// Virtual Functions from frc2::Command

void LogVisionCommand::Initialize() {
    // Start the timer
    m_timer.Reset();
    m_timer.Start();

    // Open the CSV log file
    const char* const RESULT_FILENAME = "LogVisionCommand.csv";
    m_csv_log_file.OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "LogVisionCommand::Initializ() - Failed to open log file\n";
        return;
    }

    m_csv_log_file << "SwerveFollower" << "\n";

    // Log the parameters we are using
    m_csv_log_file << "Parameters" << "\n";
    m_csv_log_file << "Max Velocity (m/s)" << m_parameters.m_max_velocity.value() << "\n";
    m_csv_log_file << "Max Acceleration (m/s2)" << m_parameters.m_max_acceleration.value() << "\n";
    m_csv_log_file <<"\n";

    // Log the trajectory we are following
    // m_csv_log_file << "Trajectory" << "\n";
    // LogAutoPaths();
    // m_csv_log_file <<"\n";

    // Log a heading for the data that will following in each update period
    m_csv_log_file << "Follower Data" << "\n";
    m_csv_log_file << "Time";
    m_csv_log_file << "Odo_X" << "Odo_Y" << "Odo_Rotation";
    m_csv_log_file << "Photon_X" << "Photon_Y" << "Photon_Rotation";
    // TODO I don't think we need the drive and speed for analysing vision (or maybe it affect
    // vision performance? Maybe comment out for now to reduce clutter)
    m_csv_log_file << "DriveX" << "DriveY" << "DriveRotation";
    m_csv_log_file << "SpeedX" << "SpeedY" << "SpeedRotation";
    m_csv_log_file << "Ambiguity";

    // Write the CSV header row columns for the manipulator values if required
    if (m_manipulator != nullptr) {
        m_csv_log_file << "State" << "IntakeState" << "Pivot" << "Shooter" << "Diverter";
    }

    m_csv_log_file <<"\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogVisionCommand::Execute() {
    // Get the pose and log it and the time
    frc::Pose2d pose = m_drivebase->GetOdometry()->GetPose();
    // frc::Pose2d vision_pose = m_vision->GetPose();
    // TODO This vision pose is from the pose estimator, not the last Photon update. It should
    // be logged, but we need the last Photon pose too (which can be null).
    LogPose(pose, m_drivebase->GetVision().GetPose());
}

bool LogVisionCommand::IsFinished() {
    // This command never finished. It must be interupted.
    return false;
}

void LogVisionCommand::End(bool interupted) {

    // Add a blank line to the log file to separate from any other data that may be appended
    m_csv_log_file <<"\n";

    // Close the file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();
}


//==============================================================================
// Logging Details

void LogVisionCommand::LogPose(frc::Pose2d odo_pose, frc::Pose2d vision_pose) {
    m_csv_log_file << m_timer.Get().value();
    m_csv_log_file << odo_pose.X().value() << odo_pose.Y().value() << odo_pose.Rotation().Degrees().value();
    m_csv_log_file << vision_pose.X().value() << vision_pose.Y().value() << vision_pose.Rotation().Degrees().value();
    const frc::ChassisSpeeds& drive_speeds = m_drivebase->GetDriveRobotSpeed();
    m_csv_log_file << drive_speeds.vx.value() << drive_speeds.vy.value() << ((units::degrees_per_second_t)drive_speeds.omega).value();
    frc::ChassisSpeeds current_speeds = m_drivebase->GetChassisSpeeds();
    m_csv_log_file << current_speeds.vx.value() << current_speeds.vy.value() << ((units::degrees_per_second_t)current_speeds.omega).value();

    // Write the CSV values for the manipulator values if required
    if (m_manipulator != nullptr) {
        m_csv_log_file << (int)m_manipulator->GetState() << (int)m_manipulator->GetIntakeCurrentState();
        m_csv_log_file << m_manipulator->GetPivot()->GetAngle().value();
        m_csv_log_file << m_manipulator->GetShooter()->GetSpeed().value();
        m_csv_log_file << m_manipulator->GetDiverter()->GetOutput();
    }

    // TODO Vision pose and ambiguity need to come from Vision::GetLatestPhotonPose() and/or
    // Vision::GetLatestPhotonResult()



    double ambiguity;
    units::degree_t angle;
    units::meter_t distance;
    m_drivebase->GetVision().GetSpeakerTargetInfo(ambiguity, angle, distance);
    m_csv_log_file << ambiguity;

    m_csv_log_file << "\n";
}

void LogVisionCommand::LogAutoPaths() {
    m_csv_log_file << "Time";
    m_csv_log_file << "PositionX" << "PositionY" << "Rotation";
    m_csv_log_file << "Heading" << "Distance" << "Velocity";
    m_csv_log_file << "Acceleration" << "Curvature"  << "MaxVelocity" << "\n";
    m_csv_log_file.SetPrecision(3, true);

    for (std::shared_ptr<PathPlannerPath> path : m_auto_paths) {
        const frc::ChassisSpeeds starting_speeds;
        const frc::Rotation2d starting_rotation;
        PathPlannerTrajectory* trajectory = new PathPlannerTrajectory(path, starting_speeds, starting_rotation);
        
        for (const PathPlannerTrajectory::State& point : trajectory->getStates()) {
            m_csv_log_file << point.time.value();
            m_csv_log_file << point.position.X().value() << point.position.Y().value() << point.targetHolonomicRotation.Degrees().value();
            m_csv_log_file << point.heading.Degrees().value() << point.deltaPos.value() << point.velocity.value();
            m_csv_log_file << point.acceleration.value() << point.curvature.value() << point.constraints.getMaxVelocity().value() << "\n";
        }

        delete trajectory;
    }
}
