//==============================================================================
// AutoBalanceCommand.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/SwerveDrivebase.h"
#include "../util/Logging.h"


class AutoBalanceCommand : public frc2::CommandHelper<frc2::Command, AutoBalanceCommand> {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // drivebase - Drivebase to drive
    AutoBalanceCommand(SwerveDrivebase* drivebase);


    //==========================================================================
	// Function Overrides from frc2::Command
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;
    //==========================================================================


    //==========================================================================
	// Static Setup

	// Setup the dashbaord controls
    static void SetupDashboard();

private:
    //==========================================================================
	// Private Nested Types

    enum class State {
        BeforeRamp,
        AfterAngle,
        Slow,
        Reversing,
        Done
    };


    //==========================================================================
	// Implementation

    void EnterState(State state);

    const char* StateName(State start);


    //==========================================================================
	// Member Variables

    SwerveDrivebase* m_drivebase;

    State m_state;
    frc::Timer m_timeout_timer;
    frc::Timer m_state_timer;
    units::degree_t m_start_angle;

    Logging::CsvFile m_csv_log_file;    // CSV log file being written to
};