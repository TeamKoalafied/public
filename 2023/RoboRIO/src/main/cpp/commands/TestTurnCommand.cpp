//==============================================================================
// TestTurnCommand.h
//==============================================================================

#include "TestTurnCommand.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

const double DEFAULT_SPEED = 1.5;
const double DEFAULT_BEFORE_TIME = 1;
const double DEFAULT_AFTER_TIME = 1;
const double DEFAULT_TURN_ANGLE = 90;

//==============================================================================
// Construction

TestTurnCommand::TestTurnCommand(SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;
    AddRequirements(drivebase);
}


//==============================================================================
// frc::Command Virtual Functions

void TestTurnCommand::Initialize() {
    // Start the timer and record that we have not turned yet
    m_speed = units::meters_per_second_t(frc::SmartDashboard::GetNumber("Speed (mps)", DEFAULT_SPEED));
    m_time_before_turn = units::second_t(frc::SmartDashboard::GetNumber("Time before turn (s)", DEFAULT_BEFORE_TIME));
    m_time_after_turn = units::second_t(frc::SmartDashboard::GetNumber("Time after turn (s)", DEFAULT_AFTER_TIME));
    m_turn = frc::Rotation2d(units::degree_t(frc::SmartDashboard::GetNumber("Turn angle (deg)", DEFAULT_TURN_ANGLE)));
   
    m_timer.Start();
    m_turned = false;
}

void TestTurnCommand::SetupDashboard() {
    frc::SmartDashboard::PutNumber("Speed (mps)", DEFAULT_SPEED);
    frc::SmartDashboard::PutNumber("Time before turn (s)", DEFAULT_BEFORE_TIME);
    frc::SmartDashboard::PutNumber("Time after turn (s)", DEFAULT_AFTER_TIME);
    frc::SmartDashboard::PutNumber("Turn angle (deg)", DEFAULT_TURN_ANGLE);
}

void TestTurnCommand::Execute() {
    // If we haven't turned but are up to 90% of the desired speed then turn now
    if (!m_turned) {
        if (m_timer.Get() > m_time_before_turn) {
            m_turned = true;
            m_timer.Reset();
            //std::cout << "Turning after " << m_timer.Get().value() << "s\n";
        }
    }


    if (!m_turned) {
        m_drivebase->Drive(m_speed, 0.0_mps, 0.0_rad_per_s, false);
    }
    else {
        m_drivebase->Drive(m_speed*m_turn.Cos(), m_speed*m_turn.Sin(), 0.0_rad_per_s, false);
    }
}

bool TestTurnCommand::IsFinished() {
    // If we exceed a short timeout then stop as something is probably wrong
    const units::second_t TIMEOUT = 5_s;
    if (m_turned) {
        if (m_timer.Get() > m_time_after_turn) {
            return true;
        }
    }
    if (m_timer.HasElapsed(TIMEOUT)) {
        std::cout << "TestTurnCommand timed out after " << m_timer.Get().value() << "s\n";
        return true;
    }

    // // If we have turned and are up to 90% of the desired speed then stop
    // units::meters_per_second_t speed = frc::SmartDashboard::GetNumber("Test Turn Speed (mps)", 0.5) * 1_mps;
    // if (m_turned) {
    //     if (m_drivebase->GetChassisSpeeds().vy > speed * 0.9) {
    //         std::cout << "TestTurnCommand finished after " << m_timer.Get().value() << "s\n";
    //         return true;
    //     }
    // }

    return false;
}
