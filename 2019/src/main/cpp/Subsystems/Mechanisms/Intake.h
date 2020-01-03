//==============================================================================
// Intake.h
//==============================================================================

#ifndef Intake_H
#define Intake_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
}

// The Intake mechanism is part of the Manipulator subsystem. It controls the
// position (in, out, vertical) and roller roation of the roller intake for
// grabbing the ball.
class Intake  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Intake();

    // Destructor
    virtual ~Intake();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates for the Intake
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Operations

    // Positions for the intake roller
    enum IntakePosition
    {
       In,         // Roller is fully retracted
       Out,        // Roller is fully extended
       Vertical,   // Roller is vertical
       Moving      // Roller is moving between positions
    };

    //
    // Get the current position of the intake roller
    //
    // Returns the current position of the intake roller. Can be IntakePosition::Moving
    // if the roller has not reached its set postion.   
    IntakePosition GetIntakePosition();

    // Drive the intake to a given position
    //
    // position - Position to drive to
    void SetIntakePosition(IntakePosition position);

//    void RaiseIntake(double motorspeed);
    
//    void LowerIntake(double motorspeed);

    // Start the intake roller
    //
    // reverse - Drive the roller in reverse
    void StartIntakeRoller(bool reverse);

    // Stop the intake roller
    void StopIntakeRoller();

    // Set the speed for the intake roller to run at. Applies next time
    // StartIntakeRoller() is called.
    //
    // speed - Speed to run [-1, 1]
    void SetIntakeRollerVelocity(double speed);

    // Convert Encoder Steps to Intake Rotation Angle
    double GetIntakeAngleDegrees();

    // Drive the intake to the given angle in degrees
    //
    // angle_degrees - Angle to drive the intake to in degrees
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void DriveToAngleDegrees(double angle_degrees, double velocity_factor);

    // Get whether the intake is set to a closed loop angle
    bool IsAngleSet();

    // Manually drive the intake at a given percentage of motor output. The intake will not
    // drive past its end limits.
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
    // Control Implementation
    
    // Perform testing of the Intake using the joystick. This function is only for testing the
    // Intake and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveAngle(ControlMode control_mode, double extension_angle);

    // Check the roller current and stop the motors if high current is detected
    // indicating a ball has been grabbed.
    void CheckRollerCurrent();

    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);

    //==========================================================================
    // Member Variables

    TalonSRX* m_intake_retract_controller;  // this controls the retract mechanism
    double m_intake_rotation_set_angle; // "Set" position of the intake, or kIntakeAngleNotSet for none

    TalonSRX* m_intake_roller_controller; // this controls the roller
    double m_intake_roller_speed;

	frc::Timer m_roller_current_timer;				// Timer for stopping the rollers after high current
    bool m_roller_grabbing;                         // Roller is being driven to grab the ball
  	bool m_roller_current_limit_exceeded;			// Whether the roller current limit has been exceeded
    bool m_ball_loaded;                             // A ball has been loaded in the intake

    static const double kIntakeDegreesPerEncoder;

    // The starting position is for the intake to be retracted ('In'), so this is angle 0.
    // Other angles are from measurement on the robot.
    static constexpr double kIntakeInAngleDegrees = 0.0;
    static constexpr double kIntakeVerticalAngleDegrees = -68.0;
    static constexpr double kIntakeOutAngleDegrees = -165.0;

    // The forward as reverse limits and the end positions. Note there are physical stops at these positions.
    static constexpr double kIntakeForwardLimitDegrees = kIntakeInAngleDegrees;
    static constexpr double kIntakeReverseLimitDegrees = kIntakeOutAngleDegrees;

    // Tolerance (-/+) for how close to the angle the intake need to be to match an IntakePosition value
    static constexpr double kIntakeToleranceAngleDegrees = 10.0;


    static constexpr double kIntakeAngleNotSet = -1000.0; // Indicates that the intake is not set

    static constexpr double kIntakeRollerSpeedDef = -0.4;

};

#endif  // Intake_H
