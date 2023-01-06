//==============================================================================
// Winch.h
//==============================================================================

#ifndef Winch_H
#define Winch_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
    class Solenoid;
}

// The Winch mechanism is part of the Manipulator subsystem. It controls the winch
// for pulling the robot up when climbing. The winch consists of the following
// actuators and sensors:
//
//  - Winch motor controlled by a Talon SRX with magnetic encoder and a limit
//    switch at the bottom
//  - Brake pneumatic cylinder controlled by a single solenoid, default position is braked
//
class Winch  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Winch();

    // Destructor
    virtual ~Winch();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Winch for operation
    void Setup();

    // Shutdown the Winch
    void Shutdown();

    // Perform periodic updates for the Winch
    void Periodic();


    //==========================================================================
    // Operations

    // Manually drive the winch at a given percentage of motor output. The winch will not
    // drive past its end limits and will not drive while braked.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveWinch(double percentage_output);

    // Get the position of the winch in inches
    //
    // Return the position of the winch in inches. The start configuration is 0 and
    // extending the winch/hook upwards is positive
    double GetWinchPositionInch();

    // Set the winch brake on
    void BrakeOn();

    // Set the winch brake off
    void BrakeOff();

    // Perform testing of the winch using the joystick. This function is only for testing the
    // winch and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveWinch(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    TalonSRX* m_winch_speed_controller; // Motor controller for the winch
    frc::Solenoid* m_brake_solenoid;    // Single solenoid for braking the winch
};

#endif  // Winch_H
