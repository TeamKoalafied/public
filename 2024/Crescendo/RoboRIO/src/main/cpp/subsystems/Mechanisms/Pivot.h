//==============================================================================
// Pivot.h
//==============================================================================

#ifndef Pivot_H
#define Pivot_H

#include "../../Phoenix5Header.h"
#include "../../RobotConfiguration.h"
#include <frc/Timer.h>
#include <units/current.h>


namespace frc { 
    class XboxController;
}

// The Pivot mechanism is part of the Manipulator subsystem. It controls the
// angle of the shooter. 
class Pivot  {
public:
    //==========================================================================
    // Constants

    // Minimum and maximum angle of the pivot in degrees
    static constexpr units::degree_t kPivotMinimumAngle = 19.7_deg;
    static constexpr units::degree_t kPivotMaximumAngle = 64.0_deg;

    // Method used when driving to an angle
    enum class DriveToMethod
    {
        Position,       // Just use Position control
        MotionMagic,    // Just use Motion Magic control
        Blend           // Use a blend of position and motion magic control
    };


    //==========================================================================
    // Construction

    // Constructor
    Pivot();

    // Destructor
    virtual ~Pivot();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Pivot for operation
    void Setup();

    // Shutdown the Pivot
    void Shutdown();

    // Perform periodic updates for the Pivot
    void Periodic();


    //==========================================================================
    // State

    // Get the pivot motor current in amps
    units::ampere_t GetCurrent() const;

    // Get the pivot motor drive as a fraction [-1, 1]
    double GetOutput() const;

    // Get the pivot angle in degrees, 0 is horizontal +ve upwards
    units::degree_t GetAngle() const;

    // Get whether the pivot is currently hitting the bottom limit switch (minimum angle)
    bool GetBottomLimitSwitch() const;

    // Get whether the pivot is currently hitting the top limit switch (maximum angle)
    bool GetTopLimitSwitch() const;


    //==========================================================================
    // Operations

    // Drive the pivot to the given angle in degrees
    //
    // angle_degrees - Angle to drive the pivot to in degrees, 0 is horizontal +ve upwards
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    // drive_to_method - Method use to drive to the position 
    void DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor, DriveToMethod drive_to_method);


    //==========================================================================
    // Testing

    // Manually drive the pivot at a given percentage of motor output. The pivot will not
    // drive past its end limit switches.
    //
    // percentage_output - Percentage output to drive at. Negative is up.
    void ManualDrivePivot(double percentage_output);

    // Perform testing of the pivot using the controller. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // controller - X-Box controller to use
    void TestDrivePivot(frc::XboxController* controller);


private:
    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);

    // Convert a pivot angle in degrees to an encoder value
    //
    // angle_degrees - pivot angle in degrees, 0 is horizontal +ve upwards
    //
    // Returns an encoder position
    double ConvertAngleDegreesToEncoder(units::degree_t angle_degrees);

    // Convert an encoder value to a pivot angle in degrees
    //
    // encoder_position - an encoder position
    //
    // Returns pivot angle in degrees, 0 is horizontal +ve upwards
    units::degree_t ConvertEncoderToAngleDegrees(units::scalar_t encoder_position) const;


    //==========================================================================
    // Member Variables

    TalonSRX* m_pivot_speed_controller;         // Motor controller for the pivot
    units::degree_t m_pivot_rotation_set_angle; // "Set" position of the pivot, or kPivotAngleNotSet for none
    double m_motion_magic_velocity_factor = -1; // Currently set motion magic velocity factor

    // The JE LPG motor encoder is a quadrature encoder with 44.4 counts per revolution, which is
    // seens as 44.4 * 4 counts by the Talon.
    // The rack has 160T for a full revolution and that pinion gear has 32T.
    // Gears from motor to pivot shaft are 24T on the motor and 50T on the shaft
    // In addition we have a 1.5 fudge factor. We have no idea why this is necessary, but it
    // gives the right answers.
    static constexpr units::degree_t kPivotDegreesPerEncoder = 360.0_deg * 1.15 / (44.4 * 4.0) * (32.0/160.0) * (24.0 / 50.0);

    static constexpr units::degree_t kPivotZeroAngle = 19.7_deg; // Angle of the pivot at the zero encoder position

    static constexpr units::degree_t kPivotAngleNotSet = -1000.0_deg; // Indicates that the pivot is not set

};

#endif  // Pivot_H
