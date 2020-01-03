//==============================================================================
// Pivot.h
//==============================================================================

#ifndef Pivot_H
#define Pivot_H

#include <ctre/Phoenix.h>
#include "../../RobotConfiguration.h"
namespace frc { 
    class Joystick;
}
namespace RC = RobotConfiguration;


// The Pivot mechanism is part of the Manipulator subsystem. It controls the
// angle of the telescopic arm about the pivot point.
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

    // Shutdown the Pivot
    void Shutdown();

    // Perform periodic updates for the Pivot
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Operations

    // Get the piot angle in degrees, relative to 0 vertical, forward +ve
    double GetPivotAngleDegrees();

    // Drive the pivot to the given angle in degrees
    //
    // angle_degrees - Angle to drive the pivot to in degrees, relative to 0 vertical, forward +ve
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void DriveToAngleDegrees(double angle_degrees, double velocity_factor);

    // Get whether the pivot is set to a closed loop angle
    bool IsAngleSet();

    // Manually drive the pivot at a given percentage of motor output. The pivot will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDrivePivot(double percentage_output);

    // Set the soft limits for pivot movement so that it does not damage other mechanisms
    //
    // forward_limit_degrees - forward pivot angle limit in degrees
    // reverse_limit_degrees - reverse pivot angle limit in degrees
    void SetSoftLimitsAngleDegrees(double forward_limit_degrees, double reverse_limit_degrees);

    // Get whether the forward soft limit is currently being hit
    bool GetForwardSoftLimitHit();

    // Get whether the reverse soft limit is currently being hit
    bool GetReverseSoftLimitHit();

    // Perform testing of the pivot using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDrivePivot(frc::Joystick* joystick);

    // Rotate the pivot to a specific position for testing using close loop control
    //
    // control_mode - Talon SRX close loop control mode to use
    // extension_inch - Extension in inches to drive to
    void TestDriveAngle(ControlMode control_mode, double extension_angle);

private:
    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);

    // Convert a pivot angle in degrees to an encoder value
    //
    // angle_degrees - pivot angle in degrees, relative to 0 vertical, forward +ve
    //
    // Returns an encoder position
    double ConvertAngleDegreesToEncoder(double angle_degrees);

    // Convert an encoder value to a pivot angle in degrees
    //
    // encoder_position - an encoder position
    //
    // Returns pivot angle in degrees, relative to 0 vertical, forward +ve
    double ConvertEncoderToAngleDegrees(double encoder_position);


    //==========================================================================
    // Member Variables
    
    TalonSRX* m_pivot_speed_controller;
    double m_pivot_rotation_set_angle; // "Set" position of the pivot, relative to 0 vertical, forward +ve, or kPivotAngleNotSet for none

    bool m_forward_limit;               // Whether the forward limit switch is down
    bool m_reverse_limit;               // Whether the reverse limit switch is down

    // Pivot angle at which the encoder measures 0
    double m_pivot_angle_zero_encoder_offset_degrees;


    // Degrees of rotation per encoder count. Encoder wheel has 4096 counts per full rotation.
    // Gearing means we need to divide this by 4.
	static const double kPivotDegreesPerEncoder;
//	static constexpr double kPivotDegreesPerEncoder = 360.0 / (4096.0 * RC::kPivotGearRatio);


    static constexpr double kPivotStartDegrees = 16.7; // 24.4; // TODO Measure this

    static constexpr double kPivotAngleNotSet = -1000.0;    // Indicates that the pivot angle is not set
};

#endif  // Pivot_H
