//==============================================================================
// Wrist.h
//==============================================================================

#ifndef Wrist_H
#define Wrist_H

#include <ctre/Phoenix.h>
namespace frc {
    class Joystick;
}


// The Wrist mechanism is part of the Manipulator subsystem. It controls the
// angle of the zucc compared to the end of the telescopic arm.
class Wrist {
public:
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
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Operations

    // Get the angle of the wrist, relative to 0 along the arm, forward +ve
    double GetWristAngleDegrees();
   
    // Drive the wrist to the given angle in degrees
    //
    // angle_degrees - Angle to drive the wrist to in degrees, relative to 0 along the arm, forward +ve
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void DriveToAngleDegrees(double angle_degrees, double velocity_factor);

    // Get whether the wrist is set to a closed loop angle
    bool IsAngleSet();

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
    void TestDriveWrist(frc::Joystick* joystick);
    
    // Perform testing of the Wrist using the joystick. This function is only for testing the
    // Wrist and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveAngle(ControlMode control_mode, double extension_angle);

private:
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
    double ConvertAngleDegreesToEncoder(double angle_degrees);

    // Convert an encoder value to a wrist angle in degrees
    //
    // encoder_position - an encoder position
    //
    // Returns wrist angle in degrees, relative to 0 along the arm, forward +ve
    double ConvertEncoderToAngleDegrees(double encoder_position);

    //==========================================================================
    // Member Variables

    TalonSRX* m_wrist_speed_controller;
    double m_wrist_rotation_set_angle; // "Set" position of the wrist, or kWristAngleNotSet for none

    // Wrist angle at which the encoder measures 0
    double m_wrist_angle_zero_encoder_offset_degrees;

    static constexpr double kWristDegreesPerEncoder = (16.0 / 18.0) * 360.0 / 4096.0;

    // Maximum rotation of the wrist in degrees. This is set to a range of values for testing
    // the wrist safely. For the purposes of testing we assume a starting position of straight up
    // and zero the count there. 
    static constexpr double kWristMaximumForwardDegrees = 120.0;
    static constexpr double kWristMaximumReverseDegrees = -90.0;

    static constexpr double kWristStartDegrees = -20.0;  // -24.8;  // -32.8; // TODO Measure this

    static constexpr double kWristAngleNotSet = -1000.0; // Indicates that the wrist is not set
};

#endif  // Wrist_H
