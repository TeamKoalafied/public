//==============================================================================
// Hood.h
//==============================================================================

#ifndef Hood_H
#define Hood_H

#include <ctre/Phoenix.h>
#include "../../RobotConfiguration.h"
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
}

// The Hood mechanism is part of the Manipulator subsystem. It controls the
// Hood wheel speed.
class Hood  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Hood();

    // Destructor
    virtual ~Hood();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Hood for operation
    void Setup();

    // Shutdown the Hood
    void Shutdown();

    // Perform periodic updates for the Hood
    void Periodic();


    //==========================================================================
    // Operations

    double GetHoodAngleDegrees();

    bool SetHoodAngleDegrees(double angle_degrees);

    void OpenLoop(double speed);

    void TestDriveHood(frc::Joystick* joystick);


private:

    //==========================================================================
    // Member Variables

    TalonSRX* m_hood_speed_controller; // Motor controller for the master motor
};

#endif  // Hood_H
