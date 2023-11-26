//==============================================================================
// LogModulesCommand.h
//==============================================================================

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/SwerveDrivebase.h"
#include "../util/Logging.h"

// LogDrivebaseCommand logs the position (X, Y, rotation) of the drivebase for as long as
// the command runs.
class LogModulesCommand : public frc2::CommandHelper<frc2::CommandBase, LogModulesCommand> {
public:
    //==========================================================================
    // Constructor
    //
    // drivebase - Drivebase to log the position of
    LogModulesCommand(SwerveDrivebase* drivebase);

    //==========================================================================
    // Virtual Functions from frc2::CommandBase
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
