//==============================================================================
// Manipulator.h
//==============================================================================

#ifndef Manipulator_H
#define Manipulator_H

#include "TSingleton.h"
#include "JoystickSubsystem.h"
#include "Mechanisms/Intake.h"

class Zucc;
class Wrist;
class Arm;
class Pivot;

// The Manipulator subsystem controls the zucc, wrist, arm, pivot and intake, which
// all work together to manipulate the game pieces.
class Manipulator : public TSingleton<Manipulator>, public JoystickSubsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    Manipulator();

    // Destructor
    virtual ~Manipulator();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void Periodic() override;
    //==========================================================================

    //==========================================================================
    // Joystick Operation (from JoystickSubsystem)
    virtual void JoystickControlStarted() override;
    virtual void DoJoystickControl() override;
    virtual void JoystickControlStopped() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the pneumatics for operation
    void Setup();

    // Shutdown the pneumatics
    void Shutdown();


    //==========================================================================
    // Mechanism Access

    // Start the vacuum
    void StartVacuum();

    // Continue vacuum if it is on and an object has been detected
    void HoldVacuum();

    // Get whether the vaccum is currently holding an object
    bool GetVacuumObjectDetected();

    // Move the pivot to the vertical position
    void GotoVertical();

private:
    //==========================================================================
    // Joystick Control

    // A preset position for the manipulator arm
    struct Preset {
        double m_angle_degrees;     // Pivot angle in degrees
        double m_extension_inch;    // Arm extension in inches
        int m_level;                // The level this preset is for
    };

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::Joystick* joystick);

    // Automatically adjust the wrist position to account for changes in the pivot angle
    //
    // wrist_driven_manually - whether the wrist is being driven manually
    void DoAutoWristAdjust(bool wrist_driven_manually);

    // Update the soft limits on pivot position based on the position of the intake
    void UpdatePivotSoftLimits();

    // Enforce the maximum extension of the arm to stay within a limit distance from the frame perimeter
    //
    // Returns true if the arm extension is being limited
    bool EnforceMaxArmExtension();

    // Adjust the pivot manual drive velocity for the current arm extension
    //
    // pivot_drive - Currnet pivot drive [-1, 1]
    // auto_drive - Whether the pivot is being automatically driven (true) or manually driven (false)
    //
    // Returns an adjusted pivot drive value to use
    double AdjustPivotVelocity(double pivot_drive, bool auto_drive);

    // Drive the intake safely to a given position
    //
    // position - Position to drive to
    //
    // Returns whether it was possible to move the intake to the given position
    bool SetIntakePositionAvoidPivot(Intake::IntakePosition position);
   
    // Do automatic control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoAutomaticJoystickControl(frc::Joystick* joystick);

    // Calculate the pivot preset to automatically drive to
    //
    // pivot_drive_auto - Current auto drive signal from the joystick
    //
    // Returns the pivot preset in degrees to drive to
    const Preset* CalculatePivotAutoPreset(double pivot_drive_auto);

    // Get the array of currently applicable presets
    //
    // total_presets - Returns the number of presets in the array
    const Preset* GetPresets(int& total_presets);

    // Get the preset for a particular level
    //
    // level - Level to get the preset for
    const Preset* GetPresetForLevel(int level);


    // Automatically adjust the offset of the zucc based on the intake deployment
    void AutoAdjustZuccOffset();


    //==========================================================================
    // Member Variables

    Zucc* m_zucc;        // The zucc mechanism
    Wrist* m_wrist;        // The wrist mechanism
    Arm* m_arm;        // The arm mechanism
    Pivot* m_pivot;        // The pivot mechanism
    Intake* m_intake;        // The intake mechanism
 
    double m_wrist_offset_degrees;      // Offset of the wrist from the 'standard' position. This is set by
                                        // the user driving the wrist, or by automatic intake adjustment.
    bool m_pivot_auto_drive;            // Whether the pivot is being auto driven
    const Preset* m_auto_preset;        // The preset to drive to in auto mode (only valid if m_pivot_auto_drive is true)
    bool m_intake_deployed;             // Whether the intake is currently deployed. Used to detect changes.

    bool m_do_auto_drive;               // Whether to do auto drive

    static constexpr double kExtensionLimitFront = 52 - 15; // inches of max extension to front of robot minus 15 buffer
    static constexpr double kExtensionLimitRear = 39 - 15; // inches of max extension to rear of robot minus 15 buffer

    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
    
};

#endif  // Manipulator_H
