//==============================================================================
// Shooter.h
//==============================================================================

#ifndef Shooter_H
#define Shooter_H

#include <ctre/Phoenix.h>
#include "../../RobotConfiguration.h"
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
}

// The Shooter mechanism is part of the Manipulator subsystem. It controls the
// shooter wheel speed.
class Shooter  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Shooter();

    // Destructor
    virtual ~Shooter();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Shooter for operation
    void Setup();

    // Shutdown the Shooter
    void Shutdown();

    // Perform periodic updates for the Shooter
    void Periodic();



    //==========================================================================
    // Operations

    // Drive the shooter in open loop at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is shooting.
    void DriveShooterOpenLoop(double percentage_output);

    // Drive the shooter in closed loop to a given speed in rpm
    //
    // shooter_wheel_rpm - Target rpm for the shooter wheel shaft. Positive is shooting.
    void DriveShooterClosedLoop(double shooter_wheel_rpm);

    // Get the current speed of the shooter wheel shaft in rpm
    int GetShooterRPM();

    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveShooter(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    TalonFX* m_shooter_master_speed_controller; // Motor controller for the master motor
    TalonFX* m_shooter_slave_speed_controller;  // Motor controller for the slave motor
};

#endif  // Shooter_H
