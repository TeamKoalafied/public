//==============================================================================
// Intake.h
//==============================================================================

#ifndef Intake_H

#include "../../Phoenix5Header.h"

#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Intake mechanism is part of the Manipulator subsystem. It grabs the cones and cubes.
// The intake consists of the following actuators and sensors:
//
//  - Roller motor controlled by a Talon SRX with magnetic encoder
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

    // Manually drive the intake at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is capture balls.
    void ManualDriveIntake(double percentage_output);

    // Check if current is exceeding the limit (12.5A)
    bool HasHighCurrent();


    // Perform testing of the intake using the joystick. This function is only for testing the
    // intake and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
   void TestDriveIntake(frc::XboxController* joystick);

    double GetOutput() const;
    double GetCurrent() const;


private:
    //==========================================================================
    // Private Nested Types

    //==========================================================================
    // Member Variables

    TalonSRX* m_intake_speed_controller;  // Motor controller for the intake roller
    // TalonSRX* m_winch_controller;           // Controller for the winch motor that moves the intake sides
    frc::Timer m_intake_current_timer;      // Timer for the intake motor high current detection

    bool m_intake_high_current = false;     
};

#endif  // Intake_H
