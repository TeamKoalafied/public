//==============================================================================
// Manipulator.h
//==============================================================================

#ifndef Manipulator_H
#define Manipulator_H

#include "JoystickSubsystem.h"
#include "../HapticController.h"
#include "../TSingleton.h"
#include "../Commands/FindTargetControl.h"
#include <ctre/Phoenix.h>
#include <frc/Timer.h>

class Hood;
class Indexer;
class Intake;
class Shooter;
class Turret;
class DistanceSensor;

// The Manipulator subsystem controls all the operator parts of the robot. It consists of the
// following mechanisms.
//
//  - Intake
//  - Indexer
//  - Shooter
//  - Climber
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

    // Setup the manipulator subsystem for operation
    void Setup();

    // Shutdown the manipulator subsystem
    void Shutdown();

    // Haptic controller for operator joystick
    HapticController* GetHapticController() { return m_haptic_controller; }


    //==========================================================================
    // Mechanism Access
    // void ExtendIntake();
    // void RetractIntake();
    // void RunIndexForward();
    // void RunIndexBack();

   //==========================================================================
   // Autonomous Control


   void StartShooter() {
       ChangeState(State::Shooting);
   }

   void StartShooterManual(double target_distance_ft) {
       m_manual_shooting_distance_ft = target_distance_ft;
       ChangeState(State::ShootingManual);
   }

   void StartPrepareShooter() {
       ChangeState(State::PrepareShooting);
   }

   void StopShooter() {
       ChangeState(State::Idle);
   }

   void StartIntake() {
       ChangeState(State::Intaking);
   }

    // Get the angle of the turret in degrees TODO document convention
    double GetTurretAngleDegrees();

    // Set the current angle of the turret in degrees
    //
    // angle_degrees - Angle in degrees to move to
    // max_speed - Maximum speed to drive the turret at [0, 1]. Will go slower as it approaches.
    bool SetTurretAngleDegrees(double angle_degrees, double max_speed);


private:
    //==========================================================================
    // Private Nested Types

    // Overall operational state of the manipulator
    enum State {
        Idle,
        Intaking,
        Rejecting,
        PrepareShooting,
        Shooting,
        ShootingManual
    };

    // Setting for the shooter speed and hood angle for a particular distance to the hub
    struct ShooterSetting {
        double m_hub_distance_inch;
        double m_shooter_speed_rpm;
        double m_hood_angle_degrees;
    };



    //==========================================================================
    // Joystick Control

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::Joystick* joystick);

    // Do control of the manipulator with the joystick for shooter calibration.
    // 
    // joystick - joystick to use
    void DoShooterCalibrationJoystickControl(frc::Joystick* joystick);


    //==========================================================================
    // State Management

    void ChangeState(State new_state);

    //==========================================================================
    // Intaking State

    void EnterIntakingState();
    void LeaveIntakingState();
    void UpdateIntakingState();


    //==========================================================================
    // Shooting State

    void EnterShootingState(bool prepare = false);
    void LeaveShootingState(bool prepare = false);
    void UpdateShootingState(bool prepare = false);

    ShooterSetting GetShooterSettingForDistance(double distance_inch);

    // Update the turret to track the target while in idle mode
    void UpdateIdleTargetTracking();



    //==========================================================================
    // Member Variables
    Hood* m_hood;               // Hood mechanism
    Indexer* m_indexer;         // Indexer mechanism
    Intake* m_intake;           // Intake mechanism
    Shooter* m_shooter;         // Shooter mechanism
    Turret* m_turret;           // Turret mechanism
    DistanceSensor* m_distance_sensor;

    State m_state;                  // Current state of the mechanism
    bool m_automatic_tracking;      // Whether automatic tracking of the target is enabled

    // Shooting State
    frc::Timer m_shoot_timer;               // Timer used to time events during shooting
    bool m_shooter_on_target;               // Whether the shooter is up to speed and on target
    int m_manual_shooting_distance_ft;      // Distance for manual shooting in feet
    
    HapticController* m_haptic_controller;          // Haptic controller for operator joystick
    FindTargetControl* m_find_target_control;       // Targeting controller


    // static const double kIndexerDriveUpVelocity;      // Relative velocity for driving the balls up the indexer when shooting
    // static const double kIndexerDriveBackVelocity;    // Relative velocity for driving the balls back down the indexer when shooting
    // static const double kKickerShootTimeS;            // Time to wait for the kicker to push the ball into the shooter in seconds
    // static const double kKickerReturnTimeS;           // Time to wait for the kicker to return to its start position in seconds
    // static const double kDriveUpTimeMaxS;             // Maximum time to drive balls up the indexer when shooting in seconds
    // static const double kDriveBackTimeS;              // Time to drive the balls back down the indexer when shooting in seconds
    // static const double kShootBallDetectInches;       // Distance in inches that indicates a ball is detected in the shooter
    // static const double kShootErrorPercentage;        // Maximum prcentage shooter speed error for shooting

    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
    
};

#endif  // Manipulator_H
