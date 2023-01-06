//==============================================================================
// Intake.h
//==============================================================================

#ifndef Intake_H
#define Intake_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include "../../RobotConfiguration.h"
#include <frc/Solenoid.h>

namespace frc { 
    class Joystick;
}

// The Intake mechanism is part of the Manipulator subsystem. It grabs the balls
// from the floor and pulls them into the robot. The intake consists of the following
// actuators and sensors:
//
//  - Roller motor controlled by a Talon SRX with magnetic encoder
//  - Deploy pneumatic cylinder controlled by a single solenoid, default position is retracted
// 
class Intake  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Intake();

    // Destructor
    ~Intake();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates for the Intake
    void Periodic();


    //==========================================================================
    // Operations

    // Deploy the intake out from the robot frame ready to grab balls
    void Extend();

    // Retract the intake into the robot frame
    void Retract();

    // Run the intake for grabbing balls
    void Run();

    // Run the intake in the reverse directions. Used for clearing jammed balls.
    void RunReverse();

    // Stop running the intake
    void Stop();

    // Manually drive the intake at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveIntake(double percentage_output);

    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveIntake(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

      TalonSRX* m_intake_speed_controller;  // Motor controller for the intake roller
      frc::Solenoid* m_intake_solenoid;     // Single solenoid for extending and retracting the intake
};

#endif  // Intake_H
