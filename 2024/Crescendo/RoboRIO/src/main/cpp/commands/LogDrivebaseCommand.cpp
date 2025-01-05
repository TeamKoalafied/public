//==============================================================================
// LogDrivebaseCommand.cpp
//==============================================================================

#include "LogDrivebaseCommand.h"

#include "../subsystems/Mechanisms/Pivot.h"
#include "../subsystems/Mechanisms/Shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>


//==============================================================================
// Constructor

LogDrivebaseCommand::LogDrivebaseCommand(SwerveDrivebase* drivebase, Manipulator* manipulator) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
}


//==============================================================================
// Virtual Functions from frc2::Command

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

    // Write the CSV header row for the drivebase values
    m_csv_log_file << "Time";
    m_csv_log_file << "X" << "Y" << "Rotation" << "\n";

    // Write the CSV header row for the manipulator values if required
    if (m_manipulator != nullptr) {
        m_csv_log_file << "State" << "IntakeState" << "Pivot" << "Shooter";
    }

    // Finishe the header and set the precision for numbers
    m_csv_log_file << "\n";
    m_csv_log_file.SetPrecision(3, true);
}

void LogDrivebaseCommand::Execute() {
    // Write the CSV values for the drivebase from the pose
    frc::Pose2d pose = m_drivebase->GetPose();
    m_csv_log_file << m_timer.Get().value();
    m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value() << "\n";

    // Write the CSV values for the manipulator values if required
    if (m_manipulator != nullptr) {
        m_csv_log_file << (int)m_manipulator->GetState() << (int)m_manipulator->GetIntakeCurrentState();
        m_csv_log_file << m_manipulator->GetPivot()->GetAngle().value();
        m_csv_log_file << m_manipulator->GetShooter()->GetSpeed().value();
    }

    // Finish the data row
    m_csv_log_file << "\n";
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
