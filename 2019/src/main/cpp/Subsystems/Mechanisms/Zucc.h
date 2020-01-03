//==============================================================================
// Zucc.h
//==============================================================================

#ifndef Zucc_H
#define Zucc_H

namespace frc {
    class DigitalInput;
    class DigitalOutput;
    class Joystick;
    class Solenoid;
    class Timer;
    class Ultrasonic;
}


// The Zucc mechanism is part of the Manipulator subsystem. It controls the
// suction cups on the end of the arm, in particular the vaccum/expel air
// control and the angle of the two cups for ball or hatch.
class Zucc {
public:
    //==========================================================================
    // Construction

    // Constructor
    Zucc();

    // Destructor
    virtual ~Zucc();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Zucc for operation
    void Setup();

    // Shutdown the Zucc
    void Shutdown();

    // Perform periodic updates for the Zucc
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Zucc Operation

    // Modes for the zucc
    enum class Mode
    {
        Hatch,        // Zucc is set to grab hatches
        Ball,         // Zucc is set to grab balls
    };

    // Start the vacuum
    void StartVacuum();

    // Continue vacuum if it is on and an object has been detected
    void HoldVacuum();

    // Start the expel and run it for the short time. If this function is called
    // again repeatedly the expel still only runs for a fixed time until there is
    // a call toStartVacuum() or HoldVacuum().
    void StartExpel();

    // Get whether the vaccum is currently holding an object
    bool GetVacuumObjectDetected();

    // The what the zucc is set to grab
    Mode GetMode();

    // Set the suction cup position for grabbing the ball
    void SetBallMode();

    // Set the suction cup position for grabbing the ball
    void SetHatchMode();

    // Perform testing of the zucc using the joystick. This function is only for testing the
    // zucc and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveZucc(frc::Joystick* joystick);

private:
    //==========================================================================
    // Expel Timer

    // State values for the expel timer
    enum class ExpelTimerState {
        NotRunning,         // Timer is running
        Running,            // Timer is running but has not exceeded the expel time duration
        Exceeded            // Timer is running and has exceeded the expel time duration
    };

    // Start the expel timer running
    void StartExpelTimer();

    // Reset the expel timer
    void ResetExpelTimer();

    // Get the state of the expel timer
    ExpelTimerState GetExpelTimerState();


    //==========================================================================
    // Member Variables

    frc::Solenoid* m_continuous_power_solenoid;            // Solenoid for Zucc. Gives continuous power.
    frc::Solenoid* m_zucc_direction_solenoid;              // Solenoid for angling Zucc suction pads for hatch or ball
    frc::DigitalOutput* m_digital_output_zucc_suck;        // Digital Output for Zucc. Enables vaccumn.
    frc::DigitalOutput* m_digital_output_zucc_expel;       // Digital Output for Zucc. Disables vaccumn.
    frc::DigitalInput* m_digital_input_zucc_indicator;     // Digital Input for Zucc. Toggles suction indicator.
    frc::Ultrasonic* m_ultrasonic;                         // Digital I/O for Zucc. Distance indicator. 
    frc::Timer* m_expel_timer;                             // Timer to track how long to expel for
    bool m_expel_timer_running;                            // Whether the expel timer is running

    static constexpr double kExpelTimeS = 1.0;             // Time to run the expel for in seconds
};

#endif  // Zucc_H
