//==============================================================================
// LogDrivebaseCommand.cpp
//==============================================================================

#include "LogDrivebaseCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>


//==============================================================================
// Constructor

LogDrivebaseCommand::LogDrivebaseCommand(SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;
}


//==============================================================================
// Virtual Functions from frc2::CommandBase

void LogDrivebaseCommand::Initialize() {
    // Start the timer
    m_timer.Reset();
    m_timer.Start();

    // Open the CSV log file
//    const char* const RESULT_FILENAME = "/home/lvuser/AutoLog.csv";
    const char* const RESULT_FILENAME = "Q:/Dev/FRC/Code/AutoLog.csv";
    m_csv_log_file.Open(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "LogDrivebaseCommand::Initializ() - Failed to open log file\n";
        return;
    }

    // Write the CSV header row and set the precision for numbers
    m_csv_log_file << "Time";
    m_csv_log_file << "X" << "Y" << "Rotation" << "\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogDrivebaseCommand::Execute() {
    // Get the pose and log it and the time
    frc::Pose2d pose = m_drivebase->GetPose();
    m_csv_log_file << m_timer.Get().value();
    m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value() << "\n";
}

bool LogDrivebaseCommand::IsFinished() {
    // This command never finished. It must be interupted.
    return false;
}

void LogDrivebaseCommand::End(bool interupted) {

    m_csv_log_file << "LogDrivebaseCommand Complete" << "\n";

    // Close the file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();
}
