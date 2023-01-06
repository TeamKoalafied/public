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

class Shooter;
class Indexer;
class Winch;
class Intake;
class Kicker;

class DistanceSensor;

// The Manipulator subsystem controls all the operator parts of the robot. It consists of the
// following mechanisms.
//
//  - Intake
//  - Indexer
//  - Kicker
//  - Shooter
//  - Winch
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
    //
    // find_target_control - Controller for targeting
    void Setup(const FindTargetControl* find_target_control);

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

   void StartIntaking() {
       ChangeState(State::Intaking);
   }

   void StopIntaking() {
       ChangeState(State::Idle);
   }

   void StartShooter() {
       m_ball_shoot_count = 0;
       ChangeState(State::Shooting);
   }

   void StartPrepareShooter() {
       ChangeState(State::PrepareShooting);
   }

   void StopShooter() {
       ChangeState(State::Idle);
   }

   int BallShootCount() { return m_ball_shoot_count; }


private:
    //==========================================================================
    // Private Nested Types

    // Overall operational state ot the manipulator
    enum State {
        Idle,
        Intaking,
        Shooting,
        PrepareShooting,
        Climbing,
    };

    // State of the current shooting operation
    enum ShootingState {
        BallInKicker,
        DrivingBallsUp,
        SettlingBallsBack,
        KickingBall,
        KickerReturn
    };


    //==========================================================================
    // Joystick Control

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::Joystick* joystick);

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
    double GetShooterWheelTargetRpm();

    //==========================================================================
    // Climbing State

    void EnterClimbingState();
    void LeaveClimbingState();
    void UpdateClimbingState(bool climb_only = false);


    //==========================================================================
    // Member Variables

    Intake* m_intake;           // Intake mechanism
    Indexer* m_indexer;         // Indexer mechanism
    Kicker* m_kicker;           // Kicker mechanism
    Shooter* m_shooter;         // Shooter mechanism
    Winch* m_winch;             // Winch mechanism
    DistanceSensor* m_distanceSensor;

    State m_state;                  // Current state of the mechanism

    // Shooting State
    frc::Timer m_shoot_timer;       // Timer used to time events during shooting
    ShootingState m_shooting_state; // State in the shooting state machine
    int m_ball_shoot_count;         // Count of balls shot. Used for autonomous.

    HapticController* m_haptic_controller;          // Haptic controller for operator joystick
    const FindTargetControl* m_find_target_control; // Targeting controller


    static const double kIndexerDriveUpVelocity;      // Relative velocity for driving the balls up the indexer when shooting
    static const double kIndexerDriveBackVelocity;    // Relative velocity for driving the balls back down the indexer when shooting
    static const double kKickerShootTimeS;            // Time to wait for the kicker to push the ball into the shooter in seconds
    static const double kKickerReturnTimeS;           // Time to wait for the kicker to return to its start position in seconds
    static const double kDriveUpTimeMaxS;             // Maximum time to drive balls up the indexer when shooting in seconds
    static const double kDriveBackTimeS;              // Time to drive the balls back down the indexer when shooting in seconds
    static const double kShootBallDetectInches;       // Distance in inches that indicates a ball is detected in the shooter
    static const double kShootErrorPercentage;        // Maximum prcentage shooter speed error for shooting

    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
    
};

#endif  // Manipulator_H
