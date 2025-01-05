//==============================================================================
// Manipulator.h
//==============================================================================

#pragma once

#include "../util/HapticController.h"
#include "../util/PovFilter.h"

#include <frc/XboxController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/RunCommand.h>
#include <frc/Timer.h>

#include "../Phoenix5Header.h"

#include <units/length.h>

#include <grpl/LaserCan.h>

namespace frc
{
class ShuffleboardTab;
class SimpleWidget;
}

class ManipulatorShuffleboard;
class SwerveDrivebase;
class Intake;
class Diverter;
class Shooter;
class Lift;
class Pivot;
class Trampler;
class Winch;
namespace Logging { class CsvFile; }



// The Manipulator subsystem controls all the operator parts of the robot. It consists of the
// following mechanisms.
//
//  - Shooter
//  - Lift
//  - Diverter
//  - Intake
class Manipulator : public frc2::SubsystemBase{
public:
    //==========================================================================
    // Public Nested Types

    // Direction form the diverter roller to send the note
    enum class DiverterPosition {
        Shooter,                    // Speaker
        Lift,                       // Trap/Amp
    };

    // Overall state of the manipulator
    enum class State {
        Idle,                       // Not doing anything in particular
        IntakingShooter,            // Intaking for shooting
        IntakingTramper,            // Intaking for scoring in the amp or tramp
        Shooting,                   // Shooting with automatic angle and speed control based on distance
        ShootingManual,             // Shooting with manual angle and speed control
        IntakeAndShooting,          // Shooting with automatic angle and speed control based on distance
        SwapToShoot,                // Swapping from trapper to shooter
        SwapToTrap,                 // Swapping from shooter to trapper
    };

    // State of intake for trapper, with uses the laserCAN sensor
    enum class IntakeState {
        Start,
        LeadingEdge,
        Middle,
        TrailingEdge,
        Stopped
    };

    // State of intake for shooting, with uses current sensing
    enum class IntakeCurrentState {
        Start,              // Waiting for the initial current spike
        Sensing,            // Attempting to sense current
        CurrentSensed,      // High current sensed. Wait to drive a bit further
        Stopped             // Intake has stopped
    };

    enum class SwapState {
        Start,
        PastDiverter,
        Reinsert,
        Stopped,
    };

    //==========================================================================
    // Construction

    // Constructor
    Manipulator();

    // Destructor
    virtual ~Manipulator();


    //==========================================================================
    // frc::Subsystem Function Overrides

    // Execute periodic function of each mechanism & update widgets
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the manipulator subsystem for operation
    void Setup(SwerveDrivebase* drivebase);

    // Setup shuffleboard tab with summary data from mechanisms

    // Shutdown the manipulator subsystem
    void Shutdown();

    // Haptic controller for operator joystick
    HapticController* GetHapticController() { return m_haptic_controller; }

    // Initialise for teleop state
    void TeleopInit();

    // Initialise for disabled state
    void DisabledInit();


    //==========================================================================
    // Mechanism Access

    // Const access to the mechanisms for monitoring their state
    const Pivot* GetPivot() const { return m_pivot; }
    const Intake* GetIntake() const { return m_intake; }
    const Diverter* GetDiverter() const { return m_diverter; }
    const Shooter* GetShooter() const { return m_shooter; }
    const Lift* GetLift() const { return m_lift; }
    const Trampler* GetTrampler() const { return m_trampler; }
    const Winch* GetWinch() const { return m_winch; }
    const units::degree_t GetTargetAngle() const { return GetShooterSettingForDistance().pivot_angle; }


    //==========================================================================
    // Manipulator State

    // Get whether the intake currently has grabbed a game piece
    DiverterPosition GetDiverterPos() const { return m_diverter_position; } 

    // Get the distance that the armm is extended
    units::inch_t GetLiftExtension();

    // Get a factor to slow the drivebase down by depending on mapiluator position (raised lift)
    double GetDrivebaseSlowdown() const;

    double GetLaser1() const;
    double GetLaser2() const;

    // Get the overall state of the manipulator
    State GetState() const { return m_state; }

    // State of intake for shooting, with uses current sensing
    IntakeCurrentState GetIntakeCurrentState() const { return m_intake_current_state; }


    //==========================================================================
    // Manipulator Control

    void SetShootingDistance(units::inch_t distance_from_sub_woofer);
    units::degree_t GetManualPivotAngle() const;
    units::revolutions_per_minute_t GetManualShooterRPM() const;

    // Set the shooter RPM and pivot angle for shooting at the target a given distance away. This will
    // persist until StopPrefire() is called.
    //
    // distance_to_target - Distance to the target
    void DoTargetingPrefire(units::inch_t distance_to_target);

    // Set the shooter RPM and pivot angle for lobbing the note. This will persist until StopPrefire() is called.
    //
    // lob_distance - Distance to lob the note
    void DoLobbingPrefire(units::inch_t lob_distance);

    // Set custom shooter RPM and pivot angle. This will persist until StopPrefire() is called.
    //
    // shooter_speed - Shooter speed
    // pivot_angle - Pivot angle
    void DoCustomPrefire(units::revolutions_per_minute_t shooter_speed, units::degree_t pivot_angle);

    // Stop
    void StopPrefire();

    // Get whether the prefire shooter RPM and pivot angle have been achieved
    bool PrefireReady() const;

    // Get the prefire currently occuring
    //
    // shooter_speed - Returns the shooter speed
    // pivot_angle - Returns the pivot angle
    //
    // Returns whether prefire is occuring
    bool GetPrefire(units::revolutions_per_minute_t& shooter_speed, units::degree_t& pivot_angle) const;

    //==========================================================================
    // Autonomous Control

    void DoAutoIntake();
    void DoAutoShooting();
    void DoAutoIntakeAndShoot();
    void StopAuto();

private:
    //==========================================================================
    // Private Nested Types
    struct ShooterSetting {
        units::inch_t distance;
        units::revolutions_per_minute_t shooter_speed;
        units::degree_t pivot_angle;
    };
    //==========================================================================
    // Joystick Control

    // Control this subsystem with the joystick controller
    void DoJoystickControl();

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::XboxController* joystick);

    void DoPOVSelectControl(frc::XboxController* joystick);

    void TrapTest(frc::XboxController* joystick);

    ShooterSetting GetShooterSettingForDistance() const;

    static ShooterSetting GetShooterSettingForDistance(units::inch_t distance_from_april_tag);

    static ShooterSetting GetLobSettingForDistance(units::inch_t lob_distance);


    // Interpolate a shooter setting from a table
    //
    // table - Table of shooter settings
    // table_size - Number of entries in 'table'
    // distance - Distance to interpolate for
    //
    // Returns shooter settings for the given distance
    static ShooterSetting InterpolateShooterSettingForDistance(ShooterSetting* table, int table_length, units::inch_t distance);


    //==========================================================================
    // State Handling

    void DoState(State state);
    void EnterState();
    void UpdateState();
    void ExitState();

    void EnterIntakingShooterState();
    void UpdateIntakingShooterState();
    void ExitIntakingShooterState();

    void EnterIntakingTramperState();
    void UpdateIntakingTramperState();
    void ExitIntakingTramperState();

    void EnterShootingState(bool manual);
    void UpdateShootingState(bool manual);
    void ExitShootingState(bool manual);

    void EnterIntakeAndShootingState();
    void UpdateIntakeAndShootingState();
    void ExitIntakeAndShootingState();

    void EnterSwapToShootState();
    void UpdateSwapToShootState();
    void ExitSwapToShootState();

    void EnterSwapToTrapState();
    void UpdateSwapToTrapState();
    void ExitSwapToTrapState();

    //==========================================================================
    // Logging

    void StartLoggingIntaking(bool mech_sort = true);
    void UpdateLoggingIntaking();
    void StopLoggingIntaking(bool cancelled);
    
    void StartLoggingShooting(bool mech_sort = true);
    void UpdateLoggingShooting();
    void StopLoggingShooting(bool cancelled);

    //==========================================================================
    // Member Variables

    SwerveDrivebase* m_drivebase = nullptr;
    Intake* m_intake;        // Intake mechanism (new version)
    Diverter* m_diverter;
    Shooter* m_shooter;
    Lift* m_lift;
    Pivot* m_pivot;
    Trampler* m_trampler;
    Winch* m_winch;

    grpl::LaserCan* m_laser1;
    grpl::LaserCan* m_laser2;
   
    frc::XboxController* m_controller;              // XBox controller
    HapticController* m_haptic_controller;          // Haptic controller for operator joystick
    PovFilter m_pov_filter;                         // Filter for POV so that it only returns the four major directions

    State m_state;                                  // Overall state of the manipulator
    IntakeState m_intake_state;                     // State of intake for trapper, with uses the laserCAN sensor
    IntakeCurrentState m_intake_current_state;      // State of intake for shooting, with uses current sensing
    DiverterPosition m_diverter_position;
    SwapState m_swap_state;
    bool m_shooter_ready = false;
    bool m_shooting_manual = false;
    bool m_lift_ready = false;
    bool m_prefiring = false;
    bool m_mech_sort;                    // Whether to sort logging columns by mechanism (true), or by quantity (current, output, position etc.) 
    frc::Timer m_intake_timer;
    frc::Timer m_shooter_timer;
    frc::Timer m_intake_state_timer;
    frc::Timer m_intake_current_timer;
    frc::Timer m_swap_timer;

    Logging::CsvFile* m_intaking_csv_log_file = nullptr;
    Logging::CsvFile* m_shooting_csv_log_file = nullptr;

    ManipulatorShuffleboard* m_shuffleboard = nullptr;      // Shuffleboard controller for this manipulator subsystem

    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing]
    units::inch_t m_distance_to_target = 0_in;

    ShooterSetting m_prefire_setting;

    static constexpr units::revolutions_per_minute_t kShooterTolerance = 75_rpm;
                                                            // Tolerance for determining if the shooter is at the correct speed
    static constexpr units::degree_t kPivotTolerance = 1_deg;
                                                            // Tolerance for determining if the pivot is at the correct angle
};
