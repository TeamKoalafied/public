//==============================================================================
// Kicker.h
//==============================================================================

#ifndef Kicker_H
#define Kicker_H

#include <ctre/Phoenix.h>

namespace frc { 
    class Joystick;
    class DoubleSolenoid;
}


// The Kicker mechanism is part of the Manipulator subsystem. It controls the pneumatic
// 'kicker' that pushes the ball from the end of the indexer into the shooter. The
// kicker consists of the following actuators and sensors:
//
//  - Kick pneumatic cylinder controlled by a double solenoid, forward position is
//    shooting and the reserve position is the start (aka 'stop') position.
// 
class Kicker  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Kicker();

    // Destructor
    virtual ~Kicker();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Kicker for operation
    void Setup();

    // Shutdown the Kicker
    void Shutdown();

    // Perform periodic updates for the Kicker
    void Periodic();


    //==========================================================================
    // Operations

    // Set the kicker to the stop position (i.e. blocking the end of the indexer)
    void SetStop();

    // Set the kicker to the shoot position (i.e. pushing the ball into the shooter)
    void SetShoot();

    // Set the kicker off (i.e. not driving the cylinder in either direction)
    void SetOff();
    
    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveKicker(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    // Solenoid that controls the kicker position
    frc::DoubleSolenoid* m_kicker_double_solenoid;
};

#endif  // Kicker_H
