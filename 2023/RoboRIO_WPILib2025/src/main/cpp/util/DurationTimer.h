//==============================================================================
// DurationTimer.h
//==============================================================================

#pragma once

#include "TimerBase.h"

#include <frc/Timer.h>

// DurationTimer repeated measures some how long some repeated operation takes. The Start() is
// called before the operation and End() after the operation. 
class DurationTimer : public TimerBase {
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
    DurationTimer(const char* name, bool enabled = true, Mode mode = DefaultMode, int display_frequency = 100,
                  units::second_t warning_max = 25_ms, units::second_t warning_ave = 20_ms);


    //==========================================================================
    // Operations

    // Start the duration timer. Call this function at the start of operatation being timed.
    void Start();

    // Stop the duration timer. Call this function at the end of operatation being timed.
    void End();

private:
    //==========================================================================
    // Member Variables

    frc::Timer m_timer;              // A timer for measuring the duration time
};