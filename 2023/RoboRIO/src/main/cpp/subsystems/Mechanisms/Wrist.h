//==============================================================================
// Wrist.h
//==============================================================================

#pragma once

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <units/constants.h>

namespace frc {
    class Joystick;
}


// The Wrist mechanism is part of the Manipulator subsystem. It controls the
// angle of the intake compared to the end of the telescopic arm.
//
//  - JE LPG motor, with its integrated gearbox and encoder, controlled by a Talon SRX
//  - One limit switch connected to the Talon SRX - forward direction
//
class Wrist {
public:
    //==========================================================================
    // Constants

    // Maximum rotation of the wrist in degrees. The starting position against the
    // limit switch is 0 degrees. Intake rotates negatively from there.
    static constexpr units::degree_t kWristMaximumForwardDegrees = 0.0_deg;
    static constexpr units::degree_t kWristMaximumReverseDegrees = -105.0_deg;


    //==========================================================================
    // Construction

    // Constructor
    Wrist();

    // Destructor
    virtual ~Wrist();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Wrist for operation
    void Setup();

    // Shutdown the Wrist
    void Shutdown();

    // Perform periodic updates for the Wrist
    void Periodic();


    //==========================================================================
    // Operations

    // Get the angle of the wrist, relative to 0 along the arm, forward +ve
    units::degree_t GetWristAngleDegrees() const;
   
    // Get whether the wrist is currently hitting the 0 degree limit switch
    bool IsAtZeroLimitSwitch() const;

    // Get whether the wrist is currently hitting the forward limit switch
    bool IsAtFullLimitSwitch() const;
 
    // Drive the wrist to the given angle in degrees
    //
    // angle_degrees - Angle to drive the wrist to in degrees, relative to 0 along the arm, forward +ve
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor);

    // Get whether the wrist is set to a closed loop angle
    bool IsAngleSet();


    //==========================================================================
    // Shuffleboard & Logging

    // Get the wrist motor drive as a fraction [-1, 1]
    double GetWristDrive();

    // Get the wrist motor current in amps
    double GetWristCurrent() const;


    //==========================================================================
    // Testing

    // Manually drive the wrist at a given percentage of motor output. The wrist will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveWrist(double percentage_output);

    // Perform testing of the wrist using the joystick. This function is only for testing the
    // wrist and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveWrist(frc::XboxController* joystick);
    
    // Perform testing of the Wrist using the joystick. This function is only for testing the
    // Wrist and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveAngle(ControlMode control_mode, units::degree_t extension_angle);

    
private:
    //==========================================================================
    // Private Nested Types

    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);

    // Convert a wrist angle in degrees to an encoder value
    //
    // angle_degrees - wrist angle in degrees, relative to 0 along the arm, forward +ve
    //
    // Returns an encoder position
    double ConvertAngleDegreesToEncoder(units::degree_t angle_degrees);

    // Convert an encoder value to a wrist angle in degrees
    //
    // encoder_position - an encoder position
    //
    // Returns wrist angle in degrees, relative to 0 along the arm, forward +ve
    units::degree_t ConvertEncoderToAngleDegrees(units::scalar_t encoder_position) const;


    //==========================================================================
    // Member Variables

    TalonSRX* m_wrist_speed_controller;         // Motor controller for the wrist
    units::degree_t m_wrist_rotation_set_angle; // "Set" position of the wrist, or kWristAngleNotSet for none
    double m_motion_magic_velocity_factor = -1; // Currently set motion magic velocity factor

    // The JE LPG motor encoder is a quadrature encoder with 44.4 counts per revolution, which is
    // seens as 44.4 * 4 counts by the Talon.
    // The worm gear has a ratio os 22.5:1 (45 teeth, double start on the screw).
    // In addition we have a 1.5 fudge factor. We have no idea why this is necessary, but it
    // gives the right answers.
    static constexpr units::degree_t kWristDegreesPerEncoder = 360.0_deg * 1.5 / (44.4 * 4 * 22.5);
    static constexpr units::degree_t kWristAngleNotSet = -1000.0_deg; // Indicates that the wrist is not set
};
