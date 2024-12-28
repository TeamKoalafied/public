//==============================================================================
// LogScrubMetric.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/SwerveDrivebase.h"
#include "../util/Logging.h"

// LogScrubMetric logs the angle of each swerve module, and its target angle for as long as
// the command runs.
class LogScrubMetric : public frc2::CommandHelper<frc2::Command, LogScrubMetric> {
public:
    //==========================================================================
    // Constructor
    //
    // drivebase - Drivebase to log the position of
    LogScrubMetric(SwerveDrivebase* drivebase);

    //==========================================================================
    // Virtual Functions from frc2::Command
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;
    //==========================================================================

private:

    //==========================================================================
    // Member variable
    SwerveDrivebase* m_drivebase;       // Drivebase to log the position of
    Logging::CsvFile m_csv_log_file;    // CSV log file being written to
    frc::Timer m_timer;                 // Timer to take how long the command has been running
};
