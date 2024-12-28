//==============================================================================
// Arm.h
//==============================================================================

#ifndef Arm_H
#define Arm_H

#include <frc/XboxController.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "../../RobotConfiguration.h"
namespace frc { 
    class Joystick;
};

namespace RC = RobotConfiguration;

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

    // Perform periodic updates for the Arm
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic();


    //==========================================================================
    // Operations

    // Get the current extension of the arm in inches
    units::inch_t GetExtensionInch() const;

    // Get whether the arm is currently hitting the 0 extension limit switch 
    bool IsAtZeroLimitSwitch() const;

    // Get whether arm is currently hitting the full extension limit switch
    bool IsAtFullLimitSwitch() const;

    double GetArmDrive();
    double GetArmCurrent() const;

    // Set the current extension of the arm in inches
    //
    // extension_inch - Extension in inches to move to
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetExtensionInch(units::inch_t extension_inch, double velocity_factor);

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
    void TestDriveArm(frc::XboxController* joystick);

    // Drive the arm to a specific position for testing using close loop control
    //
    // control_mode - Talon SRX close loop control mode to use
    // extension_inch - Extension in inches to drive to
    // void TestDriveInch(ControlMode control_mode, units::inch_t extension_inch);

private:
    //==========================================================================
    // Private Nested Types

    //==========================================================================
    // Talon Setup

    // Setup the velocity and acceleration for Motion Magic
    //
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetupMotionMagic(double velocity_factor);
    

    //==========================================================================
    // Member Variables
 
	ctre::phoenix6::hardware::TalonFX* m_arm_speed_controller;				// The arm Talon SRX speed controller
    units::inch_t m_arm_extension_set_inch;                // "Set" position of the arm, or kArmExtensionNotSet for none
    double m_motion_magic_velocity_factor = -1; // Currently set motion magic velocity factor

    // 16t #25 sprocket means 4in per revolution
    // Gearbox ratio 9:1
    // Falcon has 2048 counts
	// static constexpr units::inch_t kArmInchPerEncoder = 0.25_in * 16.0 / (9.0 * RC::kTalonFXEnocderCounts);
    static constexpr units::inch_t kArmInchPerRev = 0.25_in * 16.0 / 9.0;

    static constexpr units::inch_t kArmExtensionNotSet = -1000.0_in;    // Indicates that the pivot angle is not set

};
#endif  // Arm_H
