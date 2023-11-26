//==============================================================================
// Pivot.h
//==============================================================================

#ifndef Pivot_H
#define Pivot_H

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include "../../RobotConfiguration.h"
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
namespace frc { 
    class Joystick;
}
namespace RC = RobotConfiguration;


// The Pivot mechanism is part of the Manipulator subsystem. It controls the
// angle of the arm about the pivot point.
class Pivot {
public:
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

    // Perform periodic updates for the Pivot
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic();


    //==========================================================================
    // Operations

    // Get the pivot angle in degrees, relative to 0 vertical, forward +ve
    units::degree_t GetPivotAngleDegrees() const;

    // Get whether the pivot is currently hitting the 0 degree limit switch 
    bool IsAtZeroLimitSwitch() const;
    bool IsAtFullLimitSwitch() const;

    double GetPivotDrive();
    double GetPivotCurrent() const;

    // Drive the pivot to the given angle in degrees
    //
    // angle_degrees - Angle to drive the pivot to in degrees, relative to 0 vertical, forward +ve
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void DriveToAngleDegrees(units::degree_t angle_degrees, double velocity_factor);

    // Get whether the pivot is set to a closed loop angle
    bool IsAngleSet();

    // Manually drive the pivot at a given percentage of motor output. The pivot will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDrivePivot(double percentage_output);

    // Perform testing of the pivot using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDrivePivot(frc::XboxController* joystick);

    // Rotate the pivot to a specific position for testing using close loop control
    //
    // control_mode - Talon SRX close loop control mode to use
    // extension_inch - Extension in inches to drive to
//    void TestDriveAngle(ControlMode control_mode, double extension_angle);

private:
    //==========================================================================
    // Private Nested Types

    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);

    // Convert a pivot angle in degrees to an encoder value
    //
    // angle_degrees - pivot angle in degrees, relative to 0 vertical, forward -ve
    //
    // Returns an encoder position
    double ConvertAngleDegreesToEncoder(units::degree_t angle_degrees);

    // Convert an encoder value to a pivot angle in degrees
    //
    // encoder_position - an encoder position
    //
    // Returns pivot angle in degrees, relative to 0 vertical, forward -ve
    units::degree_t ConvertEncoderToAngleDegrees(double encoder_position) const;


    //==========================================================================
    // Member Variables
    
    TalonFX* m_pivot_speed_controller;
    units::degree_t m_pivot_rotation_set_angle; // "Set" position of the pivot, relative to 0 vertical, forward +ve, or kPivotAngleNotSet for none
    double m_motion_magic_velocity_factor = -1; // Currently set motion magic velocity factor

    bool m_forward_limit;               // Whether the forward limit switch is down
    bool m_reverse_limit;               // Whether the reverse limit switch is down

    // Degrees of rotation per encoder count. Encoder wheel has 2048 counts per full rotation.
    // Gearing means we need to divide this by 150.
	static const units::degree_t kPivotDegreesPerEncoder;


    static constexpr units::degree_t kPivotAngleNotSet = -1000.0_deg;    // Indicates that the pivot angle is not set

};

#endif  // Pivot_H
