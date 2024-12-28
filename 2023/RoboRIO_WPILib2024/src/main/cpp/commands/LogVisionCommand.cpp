//==============================================================================
// LogPathPlannerCommand.cpp
//==============================================================================

#include "LogVisionCommand.h"

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
    m_csv_log_file << "FPGATime";
    m_csv_log_file << "Odo_X" << "Odo_Y" << "Odo_Rotation";
    m_csv_log_file << "Photon_X" << "Photon_Y" << "Photon_Rotation";
    // TODO I don't think we need the drive and speed for analysing vision (or maybe it affect
    // vision performance? Maybe comment out for now to reduce clutter)
    m_csv_log_file << "RawX" << "RawY" << "RawRotation";
    m_csv_log_file << "VisionTime" << "Ambiguity";

    m_csv_log_file <<"\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogVisionCommand::Execute() {
    LogPose();
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

void LogVisionCommand::LogPose() {
    frc::Pose2d odo_pose = m_drivebase->GetOdometry()->GetPose();
    
    m_csv_log_file << m_timer.Get().value() << Timer::GetFPGATimestamp().value();
    m_csv_log_file << odo_pose.X().value() << odo_pose.Y().value() << odo_pose.Rotation().Degrees().value();
    m_csv_log_file << m_drivebase->GetVision().GetPose().X().value() << 
                        m_drivebase->GetVision().GetPose().Y().value() << 
                        m_drivebase->GetVision().GetPose().Rotation().Degrees().value();

    // TODO Vision pose and ambiguity need to come from Vision::GetLatestPhotonPose() and/or
    // Vision::GetLatestPhotonResult()
    std::optional<photon::EstimatedRobotPose> raw_vision_pose = m_drivebase->GetVision().GetLatestPhotonPose();
    if (raw_vision_pose.has_value()) {
        units::degree_t angle = raw_vision_pose.value().estimatedPose.Rotation().Z();
        m_csv_log_file << raw_vision_pose.value().estimatedPose.X().value() 
                        << raw_vision_pose.value().estimatedPose.Y().value()
                        << angle.value();
        photon::PhotonPipelineResult result = m_drivebase->GetVision().GetLatestPhotonResult();
        m_csv_log_file << raw_vision_pose.value().timestamp.value() << result.GetBestTarget().poseAmbiguity;
    } else {
        m_csv_log_file << " " << " " << " " << " ";
    }
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
