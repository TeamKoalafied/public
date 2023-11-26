//==============================================================================
// LogPathPlannerCommand.cpp
//==============================================================================

#include "LogPathPlannerCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

#include <iostream>

using namespace pathplanner;


//==============================================================================
// Constructor

LogPathPlannerCommand::LogPathPlannerCommand(SwerveDrivebase* drivebase, const pathplanner::PathPlannerTrajectory& trajectory,
                                             const Parameters parameters) {
    m_drivebase = drivebase;
    m_trajectory = trajectory;
    m_parameters = parameters;

    std::cout << "LogPathPlannerCommand::LogPathPlannerCommand() Size: " << trajectory.getStates().size() << "\n";
    std::cout << "LogPathPlannerCommand::LogPathPlannerCommand() Size: " << m_trajectory.getStates().size() << "\n";
}


//==============================================================================
// Virtual Functions from frc2::CommandBase

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
    std::cout << "LogPathPlannerCommand::Initialize() Size: " << m_trajectory.getStates().size() << "\n";
    LogTrajectory(m_csv_log_file, m_trajectory);
//    m_trajectory.LogToFile(*m_csv_log_file);
    m_csv_log_file <<"\n";

    // Log a heading for the data that will following in each update period
    m_csv_log_file << "Follower Data" << "\n";
    m_csv_log_file << "Time";
    m_csv_log_file << "X" << "Y" << "Rotation";
    //m_csv_log_file << "SP Vx" << "SP Vy" << "SP Omega";
    m_csv_log_file <<"\n";
    m_csv_log_file.SetPrecision(3, true);


//    PPSwerveControllerCommand::setLoggingCallbacks(nullptr, nullptr, [this] (auto set_point) { LogSetPoint(set_point); }, nullptr);
}

void LogPathPlannerCommand::Execute() {
    // Get the pose and log it and the time
    frc::Pose2d pose = m_drivebase->GetPose();
    LogPose(pose);
    // m_csv_log_file << m_timer.Get().value();
    // m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value() << "\n";
}

bool LogPathPlannerCommand::IsFinished() {
    // This command never finished. It must be interupted.
    return false;
}

void LogPathPlannerCommand::End(bool interupted) {

    // Add a blank line to the log file to separate from any other data that may be appended
    m_csv_log_file <<"\n";

    //m_csv_log_file << "LogPathPlannerCommand Complete" << "\n";

    // Close the file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();


    //PPSwerveControllerCommand::setLoggingCallbacks(nullptr, nullptr, nullptr, nullptr);
}


//==============================================================================

void LogPathPlannerCommand::LogPose(frc::Pose2d pose) {
    m_csv_log_file << m_timer.Get().value();
    m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value() << "\n";
}

void LogPathPlannerCommand::LogSetPoint(const frc::ChassisSpeeds& set_point) {
    m_csv_log_file << set_point.vx.value() << set_point.vy.value() << set_point.omega.value() << "\n";
}

void LogPathPlannerCommand::LogTrajectory(Logging::CsvFile& csv_log_file, const PathPlannerTrajectory& trajectory) {
    std::cout << "LogPathPlannerCommand::LogTrajectory() Size: " << trajectory.getStates().size() << "\n";
    std::cout << "LogPathPlannerCommand::LogTrajectory() Size: " << m_trajectory.getStates().size() << "\n";


    csv_log_file << "Time";
    csv_log_file << "PositionX" << "PositionY";
//    csv_log_file << "VelocityX" << "VelocityY";
//    csv_log_file << "AccelerationX" << "AccelerationY";
//    csv_log_file << "Rotation" << "AngularVelocity" << "AngularAcceleration";
    csv_log_file << "Heading" << "Distance" << "Velocity";
    csv_log_file << "Acceleration" << "Curvature"  << "MaxVelocity" << "\n";
    csv_log_file.SetPrecision(3, true);

    for (const PathPlannerTrajectory::PathPlannerState& point : trajectory.getStates()) {
        csv_log_file << point.time.value();
        csv_log_file << point.pose.X().value() << point.pose.Y().value();
//        csv_log_file << point.m_velocity_vector.x.value() << point.m_velocity_vector.y.value();
//        csv_log_file << point.m_acceleration_vector.x.value() << point.m_acceleration_vector.y.value();
//        csv_log_file << point.m_rotation_degrees.value() << point.m_rotational_velocity_degrees.value() <<
//                        point.m_rotational_acceleration_degrees.value();

        units::meter_t distance = 0_m;
        units::meters_per_second_t max_velocity = 0_mps;

        csv_log_file << point.pose.Rotation().Degrees().value() << distance.value() << point.velocity.value();
        csv_log_file << point.acceleration.value() << point.curvature.value() << max_velocity.value() << "\n";
    }

}
