//==============================================================================
// Turret.h
//==============================================================================

#ifndef Turret_H
#define Turret_H

#include <ctre/Phoenix.h>
#include "../../RobotConfiguration.h"
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
}

// The Turret mechanism is part of the Manipulator subsystem. It controls the
// rotating the shooter to face the target.
class Turret  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Turret();

    // Destructor
    virtual ~Turret();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Turret for operation
    void Setup();

    // Shutdown the Turret
    void Shutdown();

    // Perform periodic updates for the Turret
    void Periodic();


    //==========================================================================
    // Operations

    // Get the angle of the turret in degrees TODO document convention
    double GetTurretAngleDegrees();

    // Set the current angle of the turret in degrees. This function drives the turret in the
    // right direction to get to the desired angle. It should be called every update period
    // until it returns true.
    //
    // angle_degrees - Angle in degrees to move to
    // max_speed - Maximum speed to drive the turret at [0, 1]. Will go slower as it approaches the required angle.
    bool SetTurretAngleDegrees(double angle_degrees, double max_speed);

    // Manually drive the turret in open loop at a given percentage of motor output. The turret will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is extend and negative is retract.
    void OpenLoop(double speed);

    // Perform testing of the turret using the joystick. This function is only for testing the
    // turret and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveTurret(frc::Joystick* joystick);


private:
    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);


    //==========================================================================
    // Member Variables

    TalonSRX* m_turret_speed_controller;    // Motor controller for the turret motor
};

#endif  // Turret_H
