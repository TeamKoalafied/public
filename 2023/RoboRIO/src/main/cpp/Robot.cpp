//==============================================================================
// Robot.h
//==============================================================================

#include "Robot.h"

#include "subsystems/Leds.h"
#include "subsystems/SwerveDrivebase.h"
#include "subsystems/Manipulator.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>

#include <iostream>

//==============================================================================
// IterativeRobotBase Robot State Functions

void Robot::RobotInit() {
    m_leds = new Leds;
    m_manipulator = new Manipulator();
    m_drivebase = new SwerveDrivebase(m_manipulator);
    m_drivebase->Setup();
    m_manipulator->Setup();

    AutonomousCommand::SetupDevAutonomousShuffleboard();
}

void Robot::RobotPeriodic() {
    // Run the scheduler, which makes all the current commands run
    frc2::CommandScheduler::GetInstance().Run();    
}

// Autonomous ------------------------------------------------------------------

void Robot::AutonomousInit() {
    // Generate and schedule the autonomous command. Note a command is always created,
    // although it may do nothing, if that is what is selected
    m_autonomous = AutonomousCommand::CreateAutonomousCommand(*m_drivebase, *m_manipulator);
    m_autonomous.Schedule();
}

void Robot::AutonomousPeriodic() {
    // Nothing required
}

// Teleop  ---------------------------------------------------------------------

void Robot::TeleopInit() {
    // Ensure that the autonomous stops when teleop starts
    if (m_autonomous) {
        m_autonomous.Cancel();
    }
}

void Robot::TeleopPeriodic() {
    switch (m_manipulator->GetGamePiece()) {
        case Manipulator::GamePiece::Cone:
            if (m_manipulator->HasGamePiece()) {
                m_leds->SetLedPattern(Leds::Pattern::ConeGrabbed);
            } else if (m_manipulator->IsIntaking()) {
                m_leds->SetLedPattern(Leds::Pattern::ConeIntaking);
            } else {
                m_leds->SetLedPattern(Leds::Pattern::Cone);
            }
            break;
        case Manipulator::GamePiece::Cube:
            m_leds->SetLedPattern(Leds::Pattern::Cube);
            if (m_manipulator->HasGamePiece()) {
                m_leds->SetLedPattern(Leds::Pattern::CubeGrabbed);
            } else if (m_manipulator->IsIntaking()) {
                m_leds->SetLedPattern(Leds::Pattern::CubeIntaking);
            } else {
                m_leds->SetLedPattern(Leds::Pattern::Cube);
            }
            break;
    }
}

// Disabled  -------------------------------------------------------------------

void Robot::DisabledInit() {
    // Ensure all commands are cancelled when we disable the robot so that they do not
    // unexpected start if we enable it again.
    frc2::CommandScheduler::GetInstance().CancelAll();

    // Tuen off the LEDS
    m_leds->SetLedPattern(Leds::Pattern::Off);
}

void Robot::DisabledPeriodic() {
    // Nothing required
}

// Test  -----------------------------------------------------------------------

void Robot::TestInit() {
    // Nothing required
}

void Robot::TestPeriodic() {
    // Nothing required
}

// Simulation  -----------------------------------------------------------------

void Robot::SimulationInit() {
    // Initialise the drivebase simulation. We do not simulate the manipulator.
    m_drivebase->SimulationInit();
}

void Robot::SimulationPeriodic() {
    // Update drivebase simulation. We do not simulate the manipulator.
    //m_drivebase->SimulationPeriodic();
}


//==============================================================================
// Robot main() Function
//==============================================================================

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
