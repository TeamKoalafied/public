//==============================================================================
// Arm.h
//==============================================================================

#ifndef Arm_H
#define Arm_H

#include <ctre/Phoenix.h>
namespace frc { 
    class Joystick;
}


// The Arm mechanism is part of the Manipulator subsystem. It controls the extention
// and retraction of the telescropic arm.
class Arm {
public:
    //==========================================================================
    // Construction

    // Constructor
    Arm();

    // Destructor
    virtual ~Arm();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Arm for operation
    void Setup();

    // Shutdown the Arm
    void Shutdown();

    // Perform periodic updates for the Arm
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Operations

    // Get the current extension of the arm in inches
    double GetExtensionInch();

    // Set the current extension of the arm in inches
    //
    // extension_inch - Extension in inches to move to
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetExtensionInch(double extension_inch, double velocity_factor);

    // Get whether the arm is set to a closed loop extension
    bool IsExtensionSet();

    // Manually drive the arm at a given percentage of motor output. The arm will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is extend and negative is retract.
    void ManualDriveArm(double percentage_output);

    // Perform testing of the arm using the joystick. This function is only for testing the
    // arm and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveArm(frc::Joystick* joystick);

    // Drive the arm to a specific position for testing using close loop control
    //
    // control_mode - Talon SRX close loop control mode to use
    // extension_inch - Extension in inches to drive to
    void TestDriveInch(ControlMode control_mode, double extension_inch);

private:
    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);
    

    //==========================================================================
    // Member Variables
 
	TalonSRX* m_arm_speed_controller;				// The arm Talon SRX speed controller
    double m_arm_extension_set_inch;                // "Set" position of the arm, or -1 for none

    // Inches of extension per encoder count. Pulley has 28 teeth at a pitch of 5mm so the
    // circumference is 140mm.
	static constexpr double kArmInchPerEncoder = 140.0 / (25.4 * 4096); // 0.001345656988 measured 0.001346
};

#endif  // Arm_H
