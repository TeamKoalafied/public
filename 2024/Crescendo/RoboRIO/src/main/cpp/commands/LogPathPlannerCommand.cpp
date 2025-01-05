//==============================================================================
// LogPathPlannerCommand.cpp
//==============================================================================

#include "LogPathPlannerCommand.h"

#include "../subsystems/Mechanisms/Diverter.h"
#include "../subsystems/Mechanisms/Pivot.h"
#include "../subsystems/Mechanisms/Shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/commands/PathfindHolonomic.h>
#include <iostream>

using namespace pathplanner;


//==============================================================================
// Construction

LogPathPlannerCommand::LogPathPlannerCommand(SwerveDrivebase* drivebase, Manipulator* manipulator, std::shared_ptr<PathPlannerPath> path,
                                             const Parameters parameters) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
    m_auto_paths.push_back(path);
    m_parameters = parameters;
}

LogPathPlannerCommand::LogPathPlannerCommand(SwerveDrivebase* drivebase, Manipulator* manipulator, 
                                             std::vector<std::shared_ptr<PathPlannerPath>> auto_paths, const Parameters parameters)
{
    m_drivebase = drivebase;
    m_manipulator = manipulator;
    m_auto_paths = auto_paths;
    m_parameters = parameters;
}

LogPathPlannerCommand::LogPathPlannerCommand(const LogPathPlannerCommand&) {
    // This copy constructor needs to exist for the code to build but should not be called. Not sure
    // what is going on.
    assert(false);
}

LogPathPlannerCommand::~LogPathPlannerCommand() {
}


//==============================================================================
// Virtual Functions from frc2::Command

void LogPathPlannerCommand::Initialize() {
    // Start the timer
    m_timer.Reset();
    m_timer.Start();

    // Open the CSV log file
    const char* const RESULT_FILENAME = "PathPlannerAutoLog.csv";
    m_csv_log_file.OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "LogPathPlannerCommand::Initializ() - Failed to open log file\n";
        return;
    }

    m_csv_log_file << "SwerveFollower" << "\n";

    // Log the parameters we are using
    m_csv_log_file << "Parameters" << "\n";
    m_csv_log_file << "Max Velocity (m/s)" << m_parameters.m_max_velocity.value() << "\n";
    m_csv_log_file << "Max Acceleration (m/s2)" << m_parameters.m_max_acceleration.value() << "\n";
    m_csv_log_file << "Direction P" << m_parameters.m_direction_p << "\n";
    m_csv_log_file << "Direction D" << m_parameters.m_direction_d << "\n";
    m_csv_log_file << "Rotation P" << m_parameters.m_rotation_p << "\n";
    m_csv_log_file << "Rotation D" << m_parameters.m_rotation_d << "\n";
    m_csv_log_file <<"\n";

    // Log the trajectory we are following
    m_csv_log_file << "Trajectory" << "\n";
    LogAutoPaths();
    m_csv_log_file <<"\n";

    // Log a heading for the data that will following in each update period
    m_csv_log_file << "Follower Data" << "\n";
    m_csv_log_file << "Time";
    m_csv_log_file << "X" << "Y" << "Rotation";
    m_csv_log_file << "DriveX" << "DriveY" << "DriveRotation";
    m_csv_log_file << "SpeedX" << "SpeedY" << "SpeedRotation";

    // Write the CSV header row columns for the manipulator values if required
    if (m_manipulator != nullptr) {
        m_csv_log_file << "State" << "IntakeState" << "Pivot" << "Shooter" << "Diverter";
    }

    m_csv_log_file <<"\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogPathPlannerCommand::Execute() {
    // Get the pose and log it and the time
    frc::Pose2d pose = m_drivebase->GetPose();
    LogPose(pose);
}

bool LogPathPlannerCommand::IsFinished() {
    // This command never finished. It must be interupted.
    return false;
}

void LogPathPlannerCommand::End(bool interupted) {

    // Add a blank line to the log file to separate from any other data that may be appended
    m_csv_log_file <<"\n";

    // Close the file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();
}


//==============================================================================
// Logging Details

void LogPathPlannerCommand::LogPose(frc::Pose2d pose) {
    m_csv_log_file << m_timer.Get().value();
    m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value();
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


    m_csv_log_file << "\n";
}

void LogPathPlannerCommand::LogAutoPaths() {
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
