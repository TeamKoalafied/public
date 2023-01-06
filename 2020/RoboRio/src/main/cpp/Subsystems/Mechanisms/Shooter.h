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
// position (in, out, vertical) and roller roation of the roller intake for
// grabbing the ball.
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

    // Manually drive the intake at a given percentage of motor output. The intake will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveShooter(double percentage_output);

    // Drive the shooter at a given speed in RPM using closed loop velocity control
    //
    // shooter_speed_rpm - RPM to drive the shooter at (this is the roller speed not the motor speed)
    void DriveShooterRpm(double shooter_speed_rpm);

    // Get the speed of the shooter in RPM (this is the roller speed not the motor speed)
    double GetShooterRpm();

    // Test whether the shooter is at a given speed within an acceptable tolerance
    //
    // desired_shooter_speed_rpm - shooter speed to test for (this is the roller speed not the motor speed)
    //
    // Returns whether the shooter is at the given speed
    bool ShooterAtSpeed(double desired_shooter_speed_rpm);

    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveShooter(frc::Joystick* joystick);
private:

    //==========================================================================
    // Member Variables

    TalonFX* m_shooter_master_speed_controller;     // Master Talon FX speed controller for the shooter
    TalonFX* m_shooter_slave_speed_controller;      // Slave Talon FX speed controller for the shooter
};

#endif  // Shooter_H
