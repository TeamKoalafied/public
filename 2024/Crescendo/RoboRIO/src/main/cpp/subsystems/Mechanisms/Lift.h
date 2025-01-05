//==============================================================================
// Lift.h
//==============================================================================

#pragma once
#include "../../Phoenix5Header.h"
#include <units/length.h>
namespace frc { 
    class XboxController;
}


class Lift {
public:
    //==========================================================================
    // Construction

    // Constructor
    Lift();

    // Destructor
    ~Lift();

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

    // Get the current extension of the arm in inches
    units::inch_t GetExtensionInch() const;

    units::inch_t GetSetExtension() const;

    // Get whether the arm is currently hitting the 0 extension limit switch 
    bool IsAtZeroLimitSwitch() const;

    // Get whether arm is currently hitting the full extension limit switch
    bool IsAtFullLimitSwitch() const;

    bool IsExtensionSet() const;

    double GetOutput() const;
    double GetCurrent() const;

    // Set the current extension of the arm in inches
    //
    // extension_inch - Extension in inches to move to
    // velocity_factor - Factor to multiply the velocity by, in the range [0.1, 1]
    void SetExtensionInch(units::inch_t extension_inch, double velocity_factor);

    void TestDriveLift(frc::XboxController* controller);

    void ManualDriveLift(double percentage_output);

private:

    void SetupMotionMagic(double velocity_factor);

    TalonSRX* m_lift_controller;

    double m_motion_magic_velocity_factor = -1;

    
    
    static constexpr double kLiftInchPerEncoder = 22.0 * 0.25 / 4096; // 22 teeth, 1/4" pitch, 4096 counts per rev at output
    const units::inch_t kArmExtensionNotSet = -1000.0_in;
    units::inch_t m_lift_extension_set_inch = kArmExtensionNotSet;
};