//==============================================================================
// LogPathPlannerCommand.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"
#include "../util/Logging.h"


#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>

// LogPathPlannerCommand logs a list of path planner paths that will be followed and then logs the position
// (X, Y, rotation) of the drivebase for as long as the command runs.
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
    // Construction

    // Single path constructor
    //
    // drivebase - Drivebase to log the position of
    // manipulator - Manipulator to log the position of. May be nullprt.
    // path - Path the drivebase will be following
    // parameters - Parameters being used to follow the trajectory
    LogPathPlannerCommand(SwerveDrivebase* drivebase, Manipulator* manipulator, std::shared_ptr<pathplanner::PathPlannerPath> path,
                          const Parameters parameters);

    // Path list constructor
    //
    // drivebase - Drivebase to log the position of
    // manipulator - Manipulator to log the position of. May be nullprt.
    // auto_paths - List of paths for the auto the drivebase will be following
    // parameters - Parameters being used to follow the trajectory
    LogPathPlannerCommand(SwerveDrivebase* drivebase, Manipulator* manipulator,
                          std::vector<std::shared_ptr<pathplanner::PathPlannerPath>> auto_paths, const Parameters parameters);

    // Copy constructor
    LogPathPlannerCommand(const LogPathPlannerCommand&);

    // Destructor
    ~LogPathPlannerCommand();


    //==========================================================================
    // Virtual Functions from frc2::Command
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;
    //==========================================================================

private:
    //==========================================================================
    // Logging Details


    // Log the given pose during path following
    //
    // pose - Pose to log
    void LogPose(frc::Pose2d pose);

    // Log the auto paths
    void LogAutoPaths();


    //==========================================================================
    // Member variable
    
    SwerveDrivebase* m_drivebase;       // Drivebase to log the position of
    Manipulator* m_manipulator;         // Manipulator to log the position of. May be nullprt.
    Parameters m_parameters;            // Parameters being used to follow the trajectory
    std::vector<std::shared_ptr<pathplanner::PathPlannerPath>> m_auto_paths;
                                        // List of paths for the auto the drivebase will be following

    Logging::CsvFile m_csv_log_file;    // CSV log file being written to
    frc::Timer m_timer;                 // Timer to take how long the command has been running
};
