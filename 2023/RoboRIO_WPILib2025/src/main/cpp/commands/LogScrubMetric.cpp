//==============================================================================
// LogDrivebaseCommand.cpp
//==============================================================================

#include "LogScrubMetric.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <iostream>


//==============================================================================
// Constructor

LogScrubMetric::LogScrubMetric(SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;
}


//==============================================================================
// Virtual Functions from frc2::Command

void LogScrubMetric::Initialize() {
    // Start the timer
    m_timer.Reset();
    m_timer.Start();

    // Open the CSV log file
//    const char* const RESULT_FILENAME = "/home/lvuser/ModulesLog.csv";
    const char* const RESULT_FILENAME = "ModulesLog.csv";
    m_csv_log_file.OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "LogDrivebaseCommand::Initialize() - Failed to open log file\n";
        return;
    }

    // Write the CSV header row and set the precision for numbers
    m_csv_log_file << "Time";
    m_csv_log_file << "LF angle" << "LF Target" << "RF angle"<< "RF Target"  << "LB angle"<< "LB Target"  << "RB angle"<< "RB Target"  << "\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogScrubMetric::Execute() {
    // Get the pose and log it and the time
    wpi::array<const SwerveModule*, 4> modules = m_drivebase->GetModules();
    m_csv_log_file << m_timer.Get().value();
    for(int i = 0; i < 4; i++){
        frc::SwerveModuleState state = modules[i]->GetState();
        frc::SwerveModuleState desired_state = modules[i]->GetDesiredState();
        m_csv_log_file << state.angle.Degrees().value() << desired_state.angle.Degrees().value();
    }
    m_csv_log_file << "\n";
}

bool LogScrubMetric::IsFinished() {
    // This command never finished. It must be interupted.
    return false;
}

void LogScrubMetric::End(bool interupted) {
    // Close the file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();
}
