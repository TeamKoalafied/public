//==============================================================================
// PeriodicTimer.h
//==============================================================================

#pragma once


#include "TimerBase.h"

#include <frc/Timer.h>

// PeriodicTimer measure how often something happens, usually on of the robot periodic functions
class PeriodicTimer : public TimerBase {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // name - Name of the timer. Shown in warnings and statistics.
    // enable - Whether the timer is enabled. If disable nothing is display on the shuffleboard of console
    // mode - Mode flags that control display
    // display_frequency - Frequency or automatic updates (either periods or milliseconds
    //      depending on the mode).
    // warning_max - Maximum time warning level
    // warning_ave - Average time warning level
    PeriodicTimer(const char* name, bool enabled = true, Mode mode = DefaultMode, int display_frequency = 100,
                  units::second_t warning_max = 25_ms, units::second_t warning_ave = 20_ms);


    //==========================================================================
    // Operations

    // Periodic operation has occurred. Call this function when the periodic event occurs. Usually
    // this will be the start (or end) of a robot periodic function.
    void PeriodUpdate();

private:
    //==========================================================================
    // Member Variables

    frc::Timer m_timer;              // A timer for measuring the time between periods
    bool m_first_period;             // Flag to indicate that the very first period is being measured 
};
