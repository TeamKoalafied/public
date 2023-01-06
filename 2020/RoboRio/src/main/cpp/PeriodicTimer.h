//==============================================================================
// PeriodicTimer.h
//==============================================================================

#ifndef SRC_PERIODICTIMER_H_
#define SRC_PERIODICTIMER_H_

#include <frc/Timer.h>



// A timer for monitoring how often the periodic functions are called
class PeriodicTimer {
public:
    //==========================================================================
    // Construction

    // Constructor
    PeriodicTimer(const char* name);

    // Destructor
    ~PeriodicTimer();

    //==========================================================================
    // Operation

    // Initialises the timer. Call this function from the frc::IterativeRobot *Init() functions.
    void Init();

    // Updates the periodic pocessing timer. Call this function at the start of frc::IterativeRobot *Periodic() functions
    void PeriodicStart();


    // Updates the timer and log results at intervals. Call this function at the end of frc::IterativeRobot *Periodic() functions
    void PeriodicEnd();

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

    const char* m_name;                     // Name of the timer
    static const int TOTAL_DELTAS = 25;     // Number of deltas accumulated before logging
    double m_period_deltas[TOTAL_DELTAS];   // List of recorded deltas (seconds)
    double m_processing_times[TOTAL_DELTAS];// List of recorded processing times (seconds)
    int m_delta_index;                      // Index of the next delta to record or -1 before the first period
    frc::Timer m_period_timer;              // A timer that measures elapsed time, between periods
    frc::Timer m_processing_timer;          // A timer that measures processing time for one periods
};

#endif /* SRC_PERIODICTIMER_H_ */
