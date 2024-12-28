//==============================================================================
// LogPathPlannerCommand.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/SwerveDrivebase.h"
#include "../util/Logging.h"


#include <pathplanner/lib/path/PathPlannerTrajectory.h>
//namespace pathplanner { class PathPlannerTrajectory; }

// LogPathPlannerCommand logs the position (X, Y, rotation) of the drivebase for as long as
// the command runs.
class LogPathPlannerCommand : public frc2::CommandHelper<frc2::Command, LogPathPlannerCommand> {
public:
    //==========================================================================
    // Public Nested Types

    // Parameters being used to follow the trajectory
    struct Parameters {
        units::meters_per_second_t m_max_velocity;              // Maximum speed (m/s)
        units::meters_per_second_squared_t m_max_acceleration;  // Maximum acceleration (m2/s)
        double m_direction_p;                                   // The P gain of the direction PID controller
        double m_direction_d;                                   // The D gain of the direction PID controller
        double m_rotation_p;                                    // The P gain of the rotation PID controller
        double m_rotation_d;                                    // The D gain of the rotation PID controller
    };

    //==========================================================================
    // Constructor
    //
    // drivebase - Drivebase to log the position of
    LogPathPlannerCommand(SwerveDrivebase* drivebase, pathplanner::PathPlannerTrajectory& trajectory, const Parameters parameters);

    //==========================================================================
    // Virtual Functions from frc2::Command
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;
    //==========================================================================

private:

    void LogPose(frc::Pose2d pose);
    void LogSetPoint(const frc::ChassisSpeeds& set_point);
    void LogTrajectory(Logging::CsvFile& csv_log_file, pathplanner::PathPlannerTrajectory& trajectory);

    //==========================================================================
    // Member variable
    SwerveDrivebase* m_drivebase;       // Drivebase to log the position of
    pathplanner::PathPlannerTrajectory m_trajectory;
    Parameters m_parameters;            // Parameters being used to follow the trajectory

    Logging::CsvFile m_csv_log_file;    // CSV log file being written to
    frc::Timer m_timer;                 // Timer to take how long the command has been running
};
