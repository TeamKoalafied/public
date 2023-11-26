//==============================================================================
// Robot.h
//==============================================================================

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Commands.h>

class Leds;
class SwerveDrivebase;
class Manipulator;


// This is the main robot class. It is very simple, because we use subsystems and commands, and
// only really has two responsibilities.
// 
//      - Creates the subsystems
//      - Create and schedule the autonomous command
class Robot : public frc::TimedRobot {
public:
    //==========================================================================
    // IterativeRobotBase Robot State Functions
    
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    //==========================================================================
    // Member Variables

    SwerveDrivebase* m_drivebase;   // Drivebase subsystem
    Manipulator* m_manipulator;      // Manipulator subsystem 
    Leds* m_leds;                   // Leds subsystem
    frc2::CommandPtr m_autonomous { std::unique_ptr<frc2::CommandBase>() };
                                    // The autonmous command that is currently running
};
