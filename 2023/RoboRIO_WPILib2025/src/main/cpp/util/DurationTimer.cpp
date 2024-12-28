//==============================================================================
// DurationTimer.cpp
//==============================================================================

#include "DurationTimer.h"

//==============================================================================
// Construction

DurationTimer::DurationTimer(const char* name, bool enabled, Mode mode, int display_frequency,
                             units::second_t warning_max, units::second_t warning_ave) :
                             TimerBase(name, enabled, mode, display_frequency, warning_max, warning_ave) {

}


//==============================================================================
// Operations

void DurationTimer::Start() {
    m_timer.Restart();
}

void DurationTimer::End() {
    units::second_t duration = m_timer.Get();
    Update(duration);
}
