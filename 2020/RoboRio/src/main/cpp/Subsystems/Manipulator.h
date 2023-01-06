//==============================================================================
// Manipulator.h
//==============================================================================

#ifndef Manipulator_H
#define Manipulator_H

#include "../TSingleton.h"
#include "JoystickSubsystem.h"
#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include <frc/DigitalOutput.h>

class Shooter;
class Indexer;
class Winch;
class Intake;
class Kicker;

class DistanceSensor;

// The Manipulator subsystem controls the ???.
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

    // Get a factor to slow down the drivebase speed by
    double GetDriveBaseSlowDownFactor();

    void ExtendIntake();
    void RetractIntake();
    void RunIndexForward();
    void RunIndexBack();
    void Shoot();

private:
    //==========================================================================
    // Private Nested Types

    // Overall operational state ot the manipulator
    enum State {
        Idle,
        Intaking,
        Shooting,
        Climbing
    };

    // State of the current shooting operation
    enum ShootingState {
        BallInKicker,
        DrivingBallsUp,
        SettlingBallsBack,
        KickingBall,
        KickerReturn
    };


	// Sample data recorded while testing shooting
	struct ShootingDataSample
	{
		double m_time_s;				// Time in seconds the sample was taken at relative to the start of the test
        ShootingState m_shooting_state; // Shooting start being entered
		double m_shooter_rpm;		    // Current shooter RPM
        double m_indexer_position_inch; // Current position of the indexer in inches
	};


    //==========================================================================
    // Joystick Control

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::Joystick* joystick);

    //==========================================================================
    // State Management

    // Change the current mehanism state to the given one. This will leave the
    // existing state and then enter the new one.
    //
    // new_state - State to change to
    void ChangeState(State new_state);


    //==========================================================================
    // Intaking State

    // Enter the intaking balls state
    void EnterIntakingState();

    // Leave the intaking balls state
    void LeaveIntakingState();

    // Do an update for the intaking balls state. Called from periodic.
    void UpdateIntakingState();


    //==========================================================================
    // Shooting State

    // Enter the shooting balls state
    void EnterShootingState();

    // Leave the shooting balls state
    void LeaveShootingState();

    // Do an update for the shooting balls state. Called from periodic.
    void UpdateShootingState();

    // Set up for logging the shooting state
    void SetupShootingLogging();

    // Log entering the given shooting state
    //
    // shooting_state - State being entered
    void LogEnterShootingState(ShootingState shooting_state);

    // Output the log of shooter state changes to the console
    void OutputShootingLog();

    // Get a shooting state enum value as a string for logging
    //
    // shooting_state - State to get the string for
    //
    // Returns a string for the state
    const char* GetShootingStateName(ShootingState shooting_state);


    //==========================================================================
    // Climbing State

    // Enter the climbing state
    void EnterClimbingState();

    // Leave the climbing state
    void LeaveClimbingState();

    // Do an update for the climbing state. Called from periodic.
    void UpdateClimbingState();


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
    frc::Timer m_shooting_timer;    // Timer used to time events during shooting
    ShootingState m_shooting_state; // State in the shooting state machine
    frc::Timer m_shooting_log_timer;// Timer used to time stamp log entries during shooting
    std::vector<ShootingDataSample> m_shooting_sample_list;	// List of data samples recorded during the test
    frc::DigitalOutput* m_shooting_test_led_output;    // Output to drive led for shooter testing

    static const double kIndexerDriveUpVelocity;      // Relative velocity for driving the balls up the indexer when shooting
    static const double kIndexerDriveBackVelocity;    // Relative velocity for driving the balls back down the indexer when shooting
    static const double kKickerShootTimeS;            // Time to wait for the kicker to push the ball into the shooter in seconds
    static const double kKickerReturnTimeS;           // Time to wait for the kicker to return to its start position in seconds
    static const double kDriveUpTimeMaxS;             // Maximum time to drive balls up the indexer when shooting in seconds
    static const double kDriveBackTimeS;              // Time to drive the balls back down the indexer when shooting in seconds
    static const double kShootBallDetectInches;       // Distance in inches that indicates a ball is detected in the shooter

    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
    
};

#endif  // Manipulator_H
