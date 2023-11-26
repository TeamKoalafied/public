//==============================================================================
// AutoBalanceCommand.cpp
//==============================================================================

#include "AutoBalanceCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

// Melbourne real charge station
const double DEFAULT_INITIAL_SPEED = 1.5;
const double DEFAULT_INITIAL_ANGLE = 18.0;
const double DEFAULT_EXTRA_TIME = 0.4;

const double DEFAULT_SLOW_SPEED = 0.3;
const double DEFAULT_STOP_ANGLE = 9.0;
const double DEFAULT_REVERSE_SPEED = 0.5;
const double DEFAULT_REVERSE_TIME = 0.2;


// // Woolongong real charge station
// const double DEFAULT_INITIAL_SPEED = 1.5;
// const double DEFAULT_INITIAL_ANGLE = 18.0;
// const double DEFAULT_EXTRA_TIME = 0.4;

// const double DEFAULT_SLOW_SPEED = 0.3;
// const double DEFAULT_STOP_ANGLE = 10.0;
// const double DEFAULT_REVERSE_SPEED = 0.5;
// const double DEFAULT_REVERSE_TIME = 0.2;

// Our wooden ramp
// 1.5initv
// 20deg
// 0.7
// 0.6sv
// 10deg
// 1.0rv

// const double DEFAULT_INITIAL_SPEED = 1.5;
// const double DEFAULT_INITIAL_ANGLE = 20.0;
// const double DEFAULT_EXTRA_TIME = 0.7;
// const double DEFAULT_SLOW_SPEED = 0.6;
// const double DEFAULT_STOP_ANGLE = 10.0;
// const double DEFAULT_REVERSE_SPEED = 1.0;
// const double DEFAULT_REVERSE_TIME = 0.1;


//==============================================================================
// Construction

AutoBalanceCommand::AutoBalanceCommand(SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;
    m_state = State::BeforeRamp;

    AddRequirements({m_drivebase});
}


//==============================================================================
// Function Overrides from frc2::CommandBase

void AutoBalanceCommand::Initialize() {
    //std::cout << "AutoBalanceCommand::Initialize()\n";


    m_timeout_timer.Reset();
    m_timeout_timer.Start();
    EnterState(State::BeforeRamp);

    m_start_angle = m_drivebase->GetRoll();

    units::meters_per_second_t initial_speed =
        units::meters_per_second_t(frc::SmartDashboard::GetNumber("Balance Initial Speed (mps)", DEFAULT_INITIAL_SPEED));

    m_drivebase->Drive(-initial_speed, 0_mps, 0_rad_per_s, false);


    // Open the CSV log file
    m_csv_log_file.OpenLogFile("AutoBalance.csv", std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "AutoBalanceCommand::Initializ() - Failed to open log file\n";
        return;
    }

    // Write command parameters
    m_csv_log_file << "AutoBalanceCommand" << "\n";
    m_csv_log_file.SetPrecision(3, true);
    m_csv_log_file << "Balance Initial Speed (mps)" << frc::SmartDashboard::GetNumber("Balance Initial Speed (mps)", DEFAULT_INITIAL_SPEED) << "\n";
    m_csv_log_file << "Balance Initial Angle (deg)" << frc::SmartDashboard::GetNumber("Balance Initial Angle (deg)", DEFAULT_INITIAL_ANGLE) << "\n";
    m_csv_log_file << "Balance Extra Time (s)" << frc::SmartDashboard::GetNumber("Balance Extra Time (s)", DEFAULT_EXTRA_TIME) << "\n";
    m_csv_log_file << "Balance Slow Speed (mps)" << frc::SmartDashboard::GetNumber("Balance Slow Speed (mps)", DEFAULT_SLOW_SPEED) << "\n";
    m_csv_log_file << "Balance Stop Angle (deg)" << frc::SmartDashboard::GetNumber("Balance Stop Angle (deg)", DEFAULT_STOP_ANGLE) << "\n";
    m_csv_log_file << "Balance Reverse Speed (mps)" << frc::SmartDashboard::GetNumber("Balance Reverse Speed (mps)", DEFAULT_REVERSE_SPEED) << "\n";
    m_csv_log_file << "Balance Reverse Time (s)" << frc::SmartDashboard::GetNumber("Balance Reverse Time (s)", DEFAULT_REVERSE_TIME) << "\n";
    m_csv_log_file << "Start Angle" << m_drivebase->GetRoll().value() << "\n";

    // Write the CSV header row
    m_csv_log_file << "Time" << "State Time" << "Angle" << "State" << "\n";
}

void AutoBalanceCommand::Execute() {
    //std::cout << "AutoBalanceCommand::Execute() state " << (int)m_state << "\n";

    switch (m_state) {
        case State::BeforeRamp: {
            units::degree_t initial_angle =
                units::degree_t(frc::SmartDashboard::GetNumber("Balance Initial Angle (deg)", DEFAULT_INITIAL_ANGLE));
            if (m_drivebase->GetRoll() - m_start_angle > initial_angle) {
                EnterState(State::AfterAngle);
            }
            break;
        }
        case State::AfterAngle: {
            units::second_t extra_time = units::second_t(frc::SmartDashboard::GetNumber("Balance Extra Time (s)", DEFAULT_EXTRA_TIME));
            if (m_state_timer.Get() > extra_time) {
                EnterState(State::Slow);
                units::meters_per_second_t slow_speed =
                    units::meters_per_second_t(frc::SmartDashboard::GetNumber("Balance Slow Speed (mps)", DEFAULT_SLOW_SPEED));
                m_drivebase->Drive(-slow_speed, 0_mps, 0_rad_per_s, false);
            }
            break;
        }
        case State::Slow: {
            units::degree_t stop_angle =
                units::degree_t(frc::SmartDashboard::GetNumber("Balance Stop Angle (deg)", DEFAULT_STOP_ANGLE));
            if (m_drivebase->GetRoll() - m_start_angle < stop_angle && m_state_timer.Get() > 0.5_s) {
                EnterState(State::Reversing);
                units::meters_per_second_t reverse_speed =
                    units::meters_per_second_t(frc::SmartDashboard::GetNumber("Balance Reverse Speed (mps)", DEFAULT_REVERSE_SPEED));
                m_drivebase->Drive(reverse_speed, 0_mps, 0_rad_per_s, false);
            }
            break;
        }
        case State::Reversing: {
            units::second_t reverse_time = units::second_t(frc::SmartDashboard::GetNumber("Balance Reverse Time (s)", DEFAULT_REVERSE_TIME));
            if (m_state_timer.Get() > reverse_time) {
                EnterState(State::Done);
                // Locking the wheels seems to break the field relative
                m_drivebase->LockWheels();
            }
            break;
        }
        case State::Done:
            break;
    }

    if (!m_csv_log_file.Fail()) {
        m_csv_log_file << m_timeout_timer.Get().value() << m_state_timer.Get().value() << StateName(m_state) << m_drivebase->GetRoll().value() << "\n";
    }
}

bool AutoBalanceCommand::IsFinished() {
    if (m_timeout_timer.Get() > 5_s) {
        if (!m_csv_log_file.Fail()) {
            m_csv_log_file << "TIMEOUT" << "\n";
        }
        return true;
    }

    return m_state == State::Done;
}

void AutoBalanceCommand::End(bool interupted) {
    std::cout << "AutoBalanceCommand::End(" << interupted << ")/n";
    if (interupted) {
        m_drivebase->Drive(0_mps, 0_mps, 0_rad_per_s, false);
    }

    if (!m_csv_log_file.Fail()) {
        std::cout << "\n";

        // Close the file. This is important as this class may not be destroyed for some time.
        m_csv_log_file.Close();
    }
}


//==============================================================================
// Static Setup


void AutoBalanceCommand::SetupDashboard() {
    frc::SmartDashboard::PutNumber("Balance Initial Speed (mps)", DEFAULT_INITIAL_SPEED);
    frc::SmartDashboard::PutNumber("Balance Initial Angle (deg)", DEFAULT_INITIAL_ANGLE);
    frc::SmartDashboard::PutNumber("Balance Extra Time (s)", DEFAULT_EXTRA_TIME);
    frc::SmartDashboard::PutNumber("Balance Slow Speed (mps)", DEFAULT_SLOW_SPEED);
    frc::SmartDashboard::PutNumber("Balance Stop Angle (deg)", DEFAULT_STOP_ANGLE);
    frc::SmartDashboard::PutNumber("Balance Reverse Speed (mps)", DEFAULT_REVERSE_SPEED);
    frc::SmartDashboard::PutNumber("Balance Reverse Time (s)", DEFAULT_REVERSE_TIME);
}


//==============================================================================
// Implementation

void AutoBalanceCommand::EnterState(State state) {
    std::cout << "Enter state " << StateName(state);
    std::cout << " Time: " << m_timeout_timer.Get().value();
    std::cout << "s InState: " << m_state_timer.Get().value() << "s\n";

    m_state = state;
    m_state_timer.Reset();
    m_state_timer.Start();
}

const char* AutoBalanceCommand::StateName(State state) {
    switch (state) {
        case State::BeforeRamp: return "BeforeRamp ";
        case State::AfterAngle: return "AfterAngle ";
        case State::Slow:       return "Slow ";
        case State::Reversing:  return "Reversing ";
        case State::Done:       return "Done ";
        default:                return "<Unknown>";
    }
}
