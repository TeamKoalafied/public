//==============================================================================
// PeriodicTimer.h
//==============================================================================

#ifndef SRC_PERIODICTIMER_H_
#define SRC_PERIODICTIMER_H_

#include <Timer.h>


// A timer for monitoring how often the periodic functions are called
class PeriodicTimer {
public:
    //==========================================================================
    // Construction

    // Constructor
    PeriodicTimer();

    // Destructor
    ~PeriodicTimer();

    //==========================================================================
    // Operation

    // Initialises the timer. Call this function from the frc::IterativeRobot *Init() functions.
    void Init();

    // Updates the timer and log results at intervals. Call this function from the frc::IterativeRobot *Periodic() functions
    void Periodic();
private:

    //==========================================================================
    // Logging

    // Log the raw deltas to the console
    void LogAllDeltas();

    // Log summary information to the console (Min, Max, Average)
    //
    // warnings_only - Only log if the values are out of the expected range
    void LogSummary(bool warnings_only);


    //==========================================================================
    // Member Variables

    static const int TOTAL_DELTAS = 25;     // Number of deltas accumulated before logging
    double m_period_deltas[TOTAL_DELTAS];   // List of recorded deltas (seconds)
    int m_delta_index;                      // Index of the next delta to record or -1 before the first period
    frc::Timer m_timer;                     // A timer that measures elapsed time

};

#endif /* SRC_PERIODICTIMER_H_ */
